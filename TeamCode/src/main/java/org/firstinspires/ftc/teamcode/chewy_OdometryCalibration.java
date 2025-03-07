package org.firstinspires.ftc.teamcode;

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
@TeleOp(name = "chewy Odometry System Calibration", group = "Calibration")
@Disabled
public class chewy_OdometryCalibration extends LinearOpMode {
    chewy_HardwareMap robot = new chewy_HardwareMap();



    //Drive motors
//    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
 //   DcMotor verticalLeft, verticalRight, horizontal;

    //IMU Sensor
  //  BNO055IMU imu;

    // right vertical with right rear motor (i.e. whatever it is named int he hardware map)
    // left vertical with left rear motor (i.e. whatever it is named int he hardware map)
    // horizontal with left front ( i.e. whatever it is named in the hardware map)


    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
   // String rfName = "frontRightDrive", rbName = "backRightDrive", lfName = "frontLeftDrive", lbName = "backLeftDrive";
   // String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = lbName;
    final double PIVOT_SPEED = 0.5;

    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
   // final double COUNTS_PER_INCH = 1293.7705323898;

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
//        initHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
//
//        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//        //Initialize IMU parameters
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        imu.initialize(parameters);
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();
        robot.imu.resetHeading();

//          left_front.setPower(0.2);
//        sleep(1000);
//        left_front.setPower(0.0);
//
//        right_front.setPower(0.2);
//        sleep(1000);
//        right_front.setPower(0.0);
//
//        left_back.setPower(0.2);
//        sleep(1000);
//        left_back.setPower(0.0);
//
//        right_back.setPower(0.2);
//        sleep(1000);
//        right_back.setPower(0.0);

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code

        while(robot.imu.getHeading() < 90 && opModeIsActive()){
//            robot.frontRightDrive.setPower(-PIVOT_SPEED);
//            robot.frontLeftDrive.setPower(-PIVOT_SPEED);
//            robot.backRightDrive.setPower(PIVOT_SPEED);
//            robot.backLeftDrive.setPower(PIVOT_SPEED);
            if(robot.imu.getHeading() < 60) {
                setPowerAll(PIVOT_SPEED, PIVOT_SPEED, -PIVOT_SPEED, -PIVOT_SPEED);
            }else{
                setPowerAll(PIVOT_SPEED/2, PIVOT_SPEED/2, -PIVOT_SPEED/2, -PIVOT_SPEED/2);
            }

            telemetry.addData("IMU Angle", robot.imu.getHeading());
            telemetry.update();
        }

        //Stop the robot
        setPowerAll(0, 0, 0, 0);
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("IMU Angle", robot.imu.getHeading());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = robot.imu.getHeading();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(robot.verticalLeft.getCurrentPosition()) + (Math.abs(robot.verticalRight.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*robot.COUNTS_PER_INCH);

        horizontalTickOffset = robot.horizontal.getCurrentPosition()/Math.toRadians(robot.imu.getHeading());

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", robot.imu.getHeading());
            telemetry.addData("Vertical Left Position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", robot.verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", robot.horizontal.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }

//    private void initHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
//        right_front = hardwareMap.dcMotor.get(rfName);
//        right_back = hardwareMap.dcMotor.get(rbName);
//        left_front = hardwareMap.dcMotor.get(lfName);
//        left_back = hardwareMap.dcMotor.get(lbName);
//
//        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
//        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
//        horizontal = hardwareMap.dcMotor.get(hEncoderName);
//
//        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
//        left_back.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        //verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        //horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//        telemetry.addData("Status", "Hardware Map Init Complete");
//        telemetry.update();
//
//    }

    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
   // private double getZAngle(){
    //return (imu.getAngularOrientation().firstAngle);
   // }

    /**
     * Sets power to all four drive motors
     * @param rf power for right front motor
     * @param rb power for right back motor
     * @param lf power for left front motor
     * @param lb power for left back motor
     */
    private void setPowerAll(double rf, double rb, double lf, double lb){
        robot.frontRightDrive.setPower(rf);
        robot.backRightDrive.setPower(rb);
        robot.frontLeftDrive.setPower(lf);
        robot.backLeftDrive.setPower(lb);
    }

}