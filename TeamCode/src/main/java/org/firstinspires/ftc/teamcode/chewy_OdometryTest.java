package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */
@TeleOp(name = "chewy Odometry Test", group = "Calibration")
public class chewy_OdometryTest extends chewy_AutonomousMethods {
    chewy_HardwareMap robot = new chewy_HardwareMap();




    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        initOdometryHardware(0, 0, 0);


        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
//            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
//            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
//            telemetry.addData("IMU Angle", robot.imu.getHeading());
            telemetry.addData("Vertical Left Position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", robot.verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", robot.horizontal.getCurrentPosition());

            telemetry.addData("X Position", robot.globalPositionUpdate.returnXCoordinate());
            telemetry.addData("Y Position", robot.globalPositionUpdate.returnYCoordinate());
            telemetry.addData("Heading", robot.globalPositionUpdate.returnOrientation());

 //           telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

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