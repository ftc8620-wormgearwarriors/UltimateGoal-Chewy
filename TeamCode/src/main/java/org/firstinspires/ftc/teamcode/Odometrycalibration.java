package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.teamcode.Robot.Drivetrain.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */
     //add in odometry encoder names
    @TeleOp(name = "Odometry System Calibration", group = "Calibration")
    @Disabled
    public class Odometrycalibration extends LinearOpMode {
        //Drive motors
        DcMotor frontRightDrive, backRightDrive, frontLeftDrive, backLeftDrive;
        //Odometry Wheels
        DcMotor verticalLeft, verticalRight, horizontal;

        //IMU Sensor
        BNO055IMU imu;

        //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
        String rfName = "frontRightDrive", rbName = "backRightDrive", lfName = "frontLeftDrive", lbName = "backLeftDrive";
        String verticalLeftEncoderName = "vle", verticalRightEncoderName = "vre", horizontalEncoderName = "he";
        final double PIVOT_SPEED = 0.5;

        //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
        final double COUNTS_PER_INCH = 1714;

        ElapsedTime timer = new ElapsedTime();

        double horizontalTickOffset = 0;

        //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
        File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
        File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

        @Override
        public void runOpMode() throws InterruptedException {
            //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
            initHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

            //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
            imu = hardwareMap.get(BNO055IMU.class, "imu");

            //Initialize IMU parameters
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu.initialize(parameters);
            telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
            telemetry.clear();

            //Odometry System Calibration Init Complete
            telemetry.addData("Odometry System Calibration Status", "Init Complete");
            telemetry.update();

            waitForStart();

//        frontLeftDrive.setPower(0.2);
//        sleep(1000);
//        frontRightDrive.setPower(0.0);
//
//        backRightDrive.setPower(0.2);
//        sleep(1000);
//        backLeftDrive.setPower(0.0);
//
//        left_back.setPower(0.2);
//        sleep(1000);
//        left_back.setPower(0.0);
//
//        right_back.setPower(1);
//        sleep(1000);
//        right_back.setPower(0.0);

            //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
            while (getZAngle() < 90 && opModeIsActive()) {
                frontRightDrive.setPower(-PIVOT_SPEED);
                backRightDrive.setPower(-PIVOT_SPEED);
                frontLeftDrive.setPower(PIVOT_SPEED);
                backLeftDrive.setPower(PIVOT_SPEED);
                if (getZAngle() < 60) {
                    setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
                } else {
                    setPowerAll(-PIVOT_SPEED / 2, -PIVOT_SPEED / 2, PIVOT_SPEED / 2, PIVOT_SPEED / 2);
                }

                telemetry.addData("IMU Angle", getZAngle());
                telemetry.update();
            }

            //Stop the robot
            setPowerAll(0, 0, 0, 0);
            timer.reset();
            while (timer.milliseconds() < 1000 && opModeIsActive()) {
                telemetry.addData("IMU Angle", getZAngle());
                telemetry.update();
            }
            //Record IMU and encoder values to calculate the constants for the global position algorithm
            double angle = getZAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
            double encoderDifference = Math.abs(verticalLeft.getCurrentPosition()) + (Math.abs(verticalRight.getCurrentPosition()));

            double verticalEncoderTickOffsetPerDegree = encoderDifference / angle;

            double wheelBaseSeparation = (2 * 90 * verticalEncoderTickOffsetPerDegree) / (Math.PI * COUNTS_PER_INCH);

            horizontalTickOffset = horizontal.getCurrentPosition() / Math.toRadians(getZAngle());

            //Write the constants to text files
            ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
            ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

            while (opModeIsActive()) {
                telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
                //Display calculated constants
                telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
                telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

                //Display raw values
                telemetry.addData("IMU Angle", getZAngle());
                telemetry.addData("Vertical Left Position", -verticalLeft.getCurrentPosition());
                telemetry.addData("Vertical Right Position", verticalRight.getCurrentPosition());
                telemetry.addData("Horizontal Position", horizontal.getCurrentPosition());
                telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

                //Update values
                telemetry.update();
            }
        }

        private void initHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName) {
            frontRightDrive = hardwareMap.dcMotor.get(rfName);
            backRightDrive = hardwareMap.dcMotor.get(rbName);
            frontLeftDrive = hardwareMap.dcMotor.get(lfName);
            backLeftDrive = hardwareMap.dcMotor.get(lbName);

            verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
            verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
            horizontal = hardwareMap.dcMotor.get(hEncoderName);

            frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

            //verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
            //horizontal.setDirection(DcMotorSimple.Direction.REVERSE);


            telemetry.addData("Status", "Hardware Map Init Complete");
            telemetry.update();

        }

        /**
         * Gets the orientation of the robot using the REV IMU
         *
         * @return the angle of the robot
         */
        private double getZAngle() {
            return (-imu.getAngularOrientation().firstAngle);
        }
        /**
         * Sets power to all four drive motors
         *
         * @param rf power for right front motor
         * @param rb power for right back motor
         * @param lf power for left front motor
         * @param lb power for left back motor
         */
        private void setPowerAll(double rf, double rb, double lf, double lb) {
            frontRightDrive.setPower(rf);
            backRightDrive.setPower(rb);
            frontLeftDrive.setPower(lf);
            backLeftDrive.setPower(lb);
        }

    }

