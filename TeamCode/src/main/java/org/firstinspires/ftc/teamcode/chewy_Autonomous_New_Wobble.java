package org.firstinspires.ftc.teamcode;
import android.graphics.Bitmap;
import android.widget.EditText;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RingDetector;
import org.firstinspires.ftc.teamcode.RingDetectorParams;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;
import java.util.Locale;

@Autonomous(name = "chewy_Autonomous_New_Wobble")
public class chewy_Autonomous_New_Wobble extends chewy_AutonomousMethods {

    private VuforiaLocalizer vuforia = null;

    @Override
    public void runOpMode() {
        //Initialize hardware map values

        //initializing odometry hardware
        Init();
        initOdometryHardware(55, 9, 0);

        //letting cam init fully and telling driver not to start
        telemetry.addData(">","DO NOT START YET");
        telemetry.update();

        //initializing camera
        initVuforia();
        telemetry.addData("Initialized", "Camera");
        telemetry.addData(">","camera ready");
        telemetry.update();


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // wait for start of opmode
        waitForStart();

        // create RingDetector object and set bitmap based on the one chosen
        RingDetector ringDetector = new RingDetector(vuforia);

        // reset cropbox for shed
        ringDetector.setCropBox(220, 380, 95, 240);


        // do method based on counting "yellow pixels"
        int nRings = 0;

        //use the ring detector to find the number of rings
        nRings = ringDetector.getNumberOfRings();

//        while (opModeIsActive()) {
//            nRings = ringDetector.getNumberOfRings();
//            telemetry.addData("RunOpMode:NumRings", nRings);
//            telemetry.update();
//        }

        //Displaying Ring Variables
        telemetry.addData("RunOpMode:NumRings", nRings);
        telemetry.update();

        // set robot speed
        double dRobotPower = 0.9;

        //driving to intermediate pos before first drop zone
        goToPostion(54 * robot.COUNTS_PER_INCH, 60 * robot.COUNTS_PER_INCH, dRobotPower, 0, 9 * robot.COUNTS_PER_INCH, false);

        //drive and turn to drop wobble goal based on # of rings
        if (nRings == 4) {
            goToPostion(24 * robot.COUNTS_PER_INCH, 134 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);
        } else if (nRings == 1) {
            goToPostion(51 * robot.COUNTS_PER_INCH, 114 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);
        } else {
            goToPostion(24 * robot.COUNTS_PER_INCH, 92 * robot.COUNTS_PER_INCH, dRobotPower,  0, 3 * robot.COUNTS_PER_INCH, false);
        }

        //open hand and move elbow to drop wobble goal
        dropLowerWobbleGoal();
        sleep(1000);

        // turn shooter first to rev up them
        robot.shooterRight.setPower(1.0) ;
        robot.shooterLeft.setPower(-0.7);

        //move to intermediate pos
        if (nRings == 4) {
            goToPostion(36 * robot.COUNTS_PER_INCH, 135 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);
        } else if (nRings == 1) {
            goToPostion(60 * robot.COUNTS_PER_INCH, 115 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);
        }

        //Drive to lanch line-shooting pos
        goToPostion(42 * robot.COUNTS_PER_INCH, 64 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);

        // positioning to set up to grab second wobble
        robot.wobbleGrabberOpenClose.setPosition(1.0);
        robot.wobbleGrabberUpDown.setPosition(0.35);

        //shoot powershot targets
        rapidFireDisks();

        //pick up ring in one ring randomization
        if ((nRings == 1) || (nRings == 4)  ) {

            //turn intake on
            intakeOneDisk();

            //move to intermediate position before picking up ring up
            goToPostion(36 * robot.COUNTS_PER_INCH, 64 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);

            //pick up ring
            goToPostion(37 * robot.COUNTS_PER_INCH, 42 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);

            //move to shooting spot
            goToPostion(42 * robot.COUNTS_PER_INCH, 64 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);

            //wait to fully get thare
            sleep(1000);

            //shoot the disk
            rapidFireDisks();

            //turn off transports
            robot.intake.setPower(0);
            robot.firstTransfer.setPosition(0);
            robot.secondTransfer.setPosition(0);
        }


//        if (nRings == 4) {
//
//            //turn intake on
//            intakeOneDisk();
//
//            //move to intermediate position before picking up ring up
//            goToPostion(36 * robot.COUNTS_PER_INCH, 64 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);
//
//            //pick up ring
//            goToPostion(37 * robot.COUNTS_PER_INCH, 46 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);
//            sleep(1000);
//
//            //move to shooting spot
//            goToPostion(42 * robot.COUNTS_PER_INCH, 64 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);
//
//            //wait to fully get thare
//            sleep(1000);
//
//            //shoot the disk
//            rapidFireOne();
//
//            //turn off transports
//            robot.intake.setPower(0);
//            robot.firstTransfer.setPosition(0);
//            robot.secondTransfer.setPosition(0);
//        }


        //for 4 and 1 rings avoid stack
        if (nRings == 4) {
            goToPostion(17 * robot.COUNTS_PER_INCH, 60 * robot.COUNTS_PER_INCH, dRobotPower, 0, 9 * robot.COUNTS_PER_INCH, false);
        }else if (nRings == 1) {
            goToPostion(17 * robot.COUNTS_PER_INCH, 60 * robot.COUNTS_PER_INCH, dRobotPower, 0, 9 * robot.COUNTS_PER_INCH, false);
        }
        //second wobble

        //goes to pickup position and grabs second goal
        goToPostion(21.5 * robot.COUNTS_PER_INCH, 37 * robot.COUNTS_PER_INCH, dRobotPower, 0, 1 * robot.COUNTS_PER_INCH, false);
        pickUpWobbleGoal();

        if (nRings == 0) {

            //drive to targetzone for 0 rings
            goToPostion(24 * robot.COUNTS_PER_INCH, 92 * robot.COUNTS_PER_INCH, dRobotPower, 90, 3 * robot.COUNTS_PER_INCH, false);
            dropWobbleGoal();

        } else if (nRings == 1){

            //drive to targetzone for 1 rings
            goToPostion(51 * robot.COUNTS_PER_INCH,116 * robot.COUNTS_PER_INCH, dRobotPower,90,3 * robot.COUNTS_PER_INCH,false);
            dropWobbleGoal();

        } else if (nRings == 4) {

            //drive to targetzone for 4 rings
            goToPostion(25 * robot.COUNTS_PER_INCH, 138 * robot.COUNTS_PER_INCH, dRobotPower, 90, 3 * robot.COUNTS_PER_INCH, false);
            dropWobbleGoal();
        }

        //Wobble arm up
        robot.wobbleGrabberUpDown.setPosition(0.55);
        robot.wobbleGrabberOpenClose.setPosition(1.0);

        //park on launch line
        goToPostion(58 * robot.COUNTS_PER_INCH, 90 * robot.COUNTS_PER_INCH, dRobotPower, 90, 3 * robot.COUNTS_PER_INCH, false);

        //Stop the thread
        robot.globalPositionUpdate.stop();
    }


    //initializes the vuforia camera detection
    void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
}










// dont read this or somthing