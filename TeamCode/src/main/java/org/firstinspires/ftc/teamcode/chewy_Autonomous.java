package org.firstinspires.ftc.teamcode;
import android.graphics.Bitmap;
import android.widget.EditText;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled

@Autonomous(name = "chewy_Autonomous")
public class chewy_Autonomous extends chewy_AutonomousMethods {

    private VuforiaLocalizer vuforia = null;

    @Override
    public void runOpMode() {
        //Initialize hardware map values

        //initializing odometry hardware
        Init();
        initOdometryHardware(40, 9, 0);

        //initializing camera
        initVuforia();
        telemetry.addData("Initialized", "Camera");
        telemetry.update();


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // wait for start of opmode
        waitForStart();

        // create RingDetector object and set bitmap based on the one chosen
        RingDetector ringDetector = new RingDetector(vuforia);

        // reset cropbox for shed
        ringDetector.setCropBox(285, 445, 190, 300);


        // do method based on counting "yellow pixels"
        int nRings = 0;

        //use the ring detector to find the number of rings
        nRings = ringDetector.getNumberOfRings();

        //Displaying Ring Variables
        telemetry.addData("RunOpMode:NumRings", nRings);
        telemetry.update();


        // set robot speed
        double dRobotPower = 0.9;

        //driving to intermediate pos before first drop zone
        goToPostion(54 * robot.COUNTS_PER_INCH, 56 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);

        //drive and turn to drop wobble goal based on # of rings
        if (nRings == 4) {
            goToPostion(24 * robot.COUNTS_PER_INCH, 135 * robot.COUNTS_PER_INCH, dRobotPower, 90, 3 * robot.COUNTS_PER_INCH, false);
        } else if (nRings == 1) {
            goToPostion(48 * robot.COUNTS_PER_INCH, 115 * robot.COUNTS_PER_INCH, dRobotPower, 90, 3 * robot.COUNTS_PER_INCH, false);
        } else {
            goToPostion(24 * robot.COUNTS_PER_INCH, 95 * robot.COUNTS_PER_INCH, dRobotPower, 90, 3 * robot.COUNTS_PER_INCH, false);
        }

        //open hand and move elbow to drop wobble goal
        dropWobbleGoal();

        // turn shooter first to rev up them
        robot.shooterRight.setPower(0.6);
        robot.shooterLeft.setPower(-0.6);

        //move to intermediate pos
        if (nRings == 4) {
            goToPostion(36 * robot.COUNTS_PER_INCH, 135 * robot.COUNTS_PER_INCH, dRobotPower, 90, 3 * robot.COUNTS_PER_INCH, false);
        } else if (nRings == 1) {
            goToPostion(60 * robot.COUNTS_PER_INCH, 115 * robot.COUNTS_PER_INCH, dRobotPower, 90, 3 * robot.COUNTS_PER_INCH, false);
        }

        //Drive to lanch line
        goToPostion(45 * robot.COUNTS_PER_INCH, 69 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);

        //shoot powershot targets
        rapidFireDisks();

        // comment this out if you dont want to do a second wobble
        if (nRings == 0) {

            //goes to pickup position and grabes goal
            goToPostion(22 * robot.COUNTS_PER_INCH, 37 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);
            pickUpWobbleGoal();

            //drive to targetzone for 0 rings
            goToPostion(24 * robot.COUNTS_PER_INCH, 95 * robot.COUNTS_PER_INCH, dRobotPower, 90, 3 * robot.COUNTS_PER_INCH, false);
            dropWobbleGoal();
        }
//        } else if (nRings == 1){
//
//            //goes to pickup position and grabes goal
//            goToPostion(22 * robot.COUNTS_PER_INCH, 37 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);
//            pickUpWobbleGoal();
//
//            //drive to targetzone for 0 rings
//            goToPostion(48 * robot.COUNTS_PER_INCH,115 * robot.COUNTS_PER_INCH, dRobotPower,90,3 * robot.COUNTS_PER_INCH,false);
//            dropWobbleGoal();
//        }

        //park on launch line
        goToPostion(40 * robot.COUNTS_PER_INCH, 84 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);

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

