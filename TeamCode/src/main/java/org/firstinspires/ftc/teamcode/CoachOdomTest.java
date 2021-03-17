package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name = "Coach Encoder Test")
public class CoachOdomTest extends chewy_AutonomousMethods {

    private VuforiaLocalizer vuforia = null;

    @Override
    public void runOpMode() {
        //Initialize hardware map values

        //initializing odometry hardware
        Init();
        initOdometryHardware(0, 0, 0);

        //letting cam init fully and telling driver not to start
        telemetry.addData(">","DO NOT START YET");
        telemetry.update();

        //initializing camera
//        initVuforia();
//        telemetry.addData("Initialized", "Camera");
//        telemetry.addData(">","camera ready");
//        telemetry.update();


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // turn off motor brakes so human can push robot easier
        robot.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // wait for start of opmode
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Vertical Left Position", robot.globalPositionUpdate.returnVerticalLeftEncoderPosition());
            telemetry.addData("Vertical Right Position", robot.globalPositionUpdate.returnVerticalRightEncoderPosition());
            telemetry.addData("Horizontal Position", robot.globalPositionUpdate.returnNormalEncoderPosition());


            telemetry.addData("X Position", robot.globalPositionUpdate.returnXCoordinate() / robot.COUNTS_PER_INCH);
            telemetry.addData("Y Position", robot.globalPositionUpdate.returnYCoordinate() / robot.COUNTS_PER_INCH);
            telemetry.addData("Heading", robot.globalPositionUpdate.returnOrientation());
            telemetry.update();
        }


    }



}










// dont read this or somthing