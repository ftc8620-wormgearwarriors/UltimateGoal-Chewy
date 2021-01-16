package org.firstinspires.ftc.teamcode;
import android.graphics.Bitmap;

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

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;
import java.util.Locale;

@Disabled
@Autonomous(name = "chewy_Autonomous_Meet3")
public class chewy_Autonomous_Meet3 extends chewy_AutonomousMethods {

    private VuforiaLocalizer vuforia = null;
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    final String VUFORIA_KEY =
            " Ac/bw0P/////AAABmdRCZF/Kqk2MjbJIs87MKVlJg32ktQ2Tgl6871UmjRacrtxKJCUzDAeC2aA4tbiTjejLjl1W6e7VgBcQfpYx2WhqclKIEkguBRoL1udCrz4OWonoLn/GCA+GntFUZN0Az+dGGYtBqcuW3XkmVNSzgOgJbPDXOf+73P5qb4/mHry0xjx3hysyAzmM/snKvGv8ImhVOVpm00d6ozC8GzvOMRF/S5Z1NBsoFls2/ul+PcZ+veKwgyPFLEFP4DXSqTeOW1nJGH9yYXSH0kfNHgGutLM5om1hAlxdP8D4XMRD2bgWXj1Md2bz+uJmr1E2ZuI7p26ZRxOIKZE9Hwpai+MW6yaJD0otF6aL9QXYaULPpWKo ";

    @Override
    public void runOpMode() {
        //Initialize hardware map values.

        //initializing odometry hardware
        Init();
        initOdometryHardware(40,  9 , 0);

        //initializing camera
        initVuforia();
        initTfod();
        telemetry.addData("Initialized", "Camera" );
        telemetry.update();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData("tfod", "activated");

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // wait for start of opmode
        waitForStart();

        //use camera to identify # of rings
        int numTries = 1;
        int numRings = numberOfRings(numTries);
        telemetry.addData("# Rings Detected", numRings);
        telemetry.update();

        //driving to intermediate pos before first drop zone
        goToPostion(54 * robot.COUNTS_PER_INCH,56 * robot.COUNTS_PER_INCH,0.8,0,3 * robot.COUNTS_PER_INCH,false);

        //drive and turn to drop wobble goal based on # of rings
        if (numRings == 4 ) {
            goToPostion(24 * robot.COUNTS_PER_INCH,135 * robot.COUNTS_PER_INCH,0.8,90,3 * robot.COUNTS_PER_INCH,false);
        } else if (numRings == 1) {
            goToPostion(48 * robot.COUNTS_PER_INCH,115 * robot.COUNTS_PER_INCH,0.8,90,3 * robot.COUNTS_PER_INCH,false);
        } else {
            goToPostion(24 * robot.COUNTS_PER_INCH,95 * robot.COUNTS_PER_INCH,0.8,90,3 * robot.COUNTS_PER_INCH,false);
        }

        //open hand and move elbow to drop wobble goal
        dropWobbleGoal();

        //move to intermedit pos
        if (numRings == 4 ) {
            goToPostion(36 * robot.COUNTS_PER_INCH,135 * robot.COUNTS_PER_INCH,0.8,90,3 * robot.COUNTS_PER_INCH,false);
        } else if (numRings == 1) {
            goToPostion(60 * robot.COUNTS_PER_INCH,115 * robot.COUNTS_PER_INCH,0.8,90,3 * robot.COUNTS_PER_INCH,false);
        }

        // turn shooter first to rev up them
        robot.shooterRight.setPower(.5);
        robot.shooterLeft.setPower(-0.5);

        //Drive to lanch line
        goToPostion(38 * robot.COUNTS_PER_INCH,69 * robot.COUNTS_PER_INCH,0.8,0,3 * robot.COUNTS_PER_INCH,false);

        //shoot powershot targets
        rapidFireDisks();

        //drive back to get wobble goal
        if ((numRings == 4) || (numRings == 1)) {
            //going to mid pos before other wobble to avoid knocking over rings
            goToPostion(20 * robot.COUNTS_PER_INCH,62 * robot.COUNTS_PER_INCH,0.8,0,3 * robot.COUNTS_PER_INCH,false);
            goToPostion(20.5 * robot.COUNTS_PER_INCH,35 * robot.COUNTS_PER_INCH,0.8,0,3 * robot.COUNTS_PER_INCH,false);
        } else {
            //skip past mid pos because of no rings
            goToPostion(20.5 * robot.COUNTS_PER_INCH,35 * robot.COUNTS_PER_INCH,0.8,0,3 * robot.COUNTS_PER_INCH,false);
        }
        pickUpWobbleGoal();

        //drive back to wobble drop zone
        if (numRings == 4 ) {
            goToPostion(24 * robot.COUNTS_PER_INCH,135 * robot.COUNTS_PER_INCH,0.8,90,3 * robot.COUNTS_PER_INCH,false);
        } else if (numRings == 1) {
            goToPostion(44 * robot.COUNTS_PER_INCH,110 * robot.COUNTS_PER_INCH,0.8,90,3 * robot.COUNTS_PER_INCH,false);
        } else {
            goToPostion(24 * robot.COUNTS_PER_INCH,95 * robot.COUNTS_PER_INCH,0.8,90,3 * robot.COUNTS_PER_INCH,false);
        }

        //open hand and move elbow to drop wobble goal
        dropWobbleGoal();

        //park on launch line
        goToPostion(36 * robot.COUNTS_PER_INCH,84 * robot.COUNTS_PER_INCH,0.8,0,3 * robot.COUNTS_PER_INCH,false);

        //Stop the thread
        robot.globalPositionUpdate.stop();
    }

    int numberOfRings(int numTries){
        int returnRings = 0;

        // try writing image to file
        captureFrameToFile();
        telemetry.addData("folder",captureDirectory);
        telemetry.update();
        sleep(1000);

        if (tfod != null) {
            for (int i = 0; i < numTries; i++) {

                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData(String.format("# objects detected (%d)", i), updatedRecognitions.size());
                    telemetry.update();
                    sleep(200);

                    // step through the list of recognitions and display boundary info.
                    int j = 0;
                    for (Recognition recognition : updatedRecognitions) {

                        //telemetry.addData(String.format("label (%d)", j), recognition.getLabel());
                        //telemetry.update();

                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                            returnRings = 1;
                            return returnRings;
                        }
                        if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                            returnRings = 4;
                            return returnRings;
                        }

                    }
                }
            }
        }
        return returnRings;
    }

    //initializes object detector (tfod)
    void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
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

    //use vuforia to capture an image that determines the number of donuts
    void captureFrameToFile() {
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>()
        {
            @Override public void accept(Frame frame)
            {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                saveBitmap(bitmap);
                if (bitmap != null) {
                    File file = new File(captureDirectory, String.format(Locale.getDefault(), "VuforiaFrame-%d.png", captureCounter++));
                    try {
                        FileOutputStream outputStream = new FileOutputStream(file);
                        try {
                            bitmap.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
                        } finally {
                            outputStream.close();
                            telemetry.log().add("captured %s", file.getName());
                        }
                    } catch (IOException e) {
                        RobotLog.ee(TAG, e, "exception in captureFrameToFile()");
                    }
                }
            }
        }));
    }

}
