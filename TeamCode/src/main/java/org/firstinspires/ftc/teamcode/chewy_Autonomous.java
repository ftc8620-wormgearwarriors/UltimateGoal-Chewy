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

@Autonomous(name = "chewy_Autonomous")
public class chewy_Autonomous extends chewy_AutonomousMethods {

    private VuforiaLocalizer vuforia = null;
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    @Override
    public void runOpMode() {
        //Initialize hardware map values

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

        // clear the data folder here so it's clean when waiting for webcam image to show up
        clearDataDirectory();

        telemetry.addData("tfod", "activated");
        telemetry.addData("capture directory", "cleared");

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // wait for start of opmode
        waitForStart();

        // capture the image from the webcam
        captureFrameToFile();
        sleep(2000);

        // now read the webcam image (this also waits for it to show up)
        Bitmap bitmapRingImage = readWebcamImage();
        saveBitmap(bitmapRingImage);

        // create RingDetector object and set bitmap based on the one chosen
        RingDetector ringDetector = new RingDetector(bitmapRingImage);

//        // default box location
//        int m_cropBoxLeft = 300;
//        int m_cropBoxRight = 420;
//        int m_cropBoxTop = 210;
//        int m_cropBoxBottom = 290;

        // reset cropbox for front room
        ringDetector.setCropBox(285, 400, 235, 315);

        // reset cropbox for shed

        // create the cropped image and display it
        Bitmap bitmapCroppedRingImage = ringDetector.createCroppedRingImage();
        saveCroppedBitmap(bitmapCroppedRingImage);

        // find number of rings and set it in edit text
        // parameters for klingensmith front room with phone light used
        int nPixelThreshold = 120;
        double dRatioThreshold2 = 2.0;
        double dRatioThreshold1 = 1.15;

//        // parameters for shed
//        int nPixelThreshold = 140;
//        double dRatioThreshold2 = 2.0;
//        double dRatioThreshold1 = 0.5;

        double dRBRatio = ringDetector.getRBRatio(bitmapCroppedRingImage, nPixelThreshold,
                dRatioThreshold1, dRatioThreshold2);

        //check ratios vs thresholds to decide on number of rings
        //if either ratio is above threshold 2, we will set to 4 rings
        //if either ratio is below threshold 1, we will set to 0 rings
        int nRings = 0;
        if (dRBRatio > dRatioThreshold2) {
            nRings = 4;
        }
        else if (dRBRatio < dRatioThreshold1) {
            nRings = 0;
        }
        else {
            nRings = 1;
        }

        //Displaying Ring Variables
        telemetry.addData("RunOpMode:RedBlueRatio", dRBRatio);
        telemetry.addData("RunOpMode:NumRings", nRings);
        telemetry.update();
        sleep(8000);

//        //driving to intermediate pos before first drop zone
//        goToPostion(54 * robot.COUNTS_PER_INCH,56 * robot.COUNTS_PER_INCH,0.8,0,3 * robot.COUNTS_PER_INCH,false);
//
//        //drive and turn to drop wobble goal based on # of rings
//        if (nRings == 4 ) {
//            goToPostion(24 * robot.COUNTS_PER_INCH,135 * robot.COUNTS_PER_INCH,0.8,90,3 * robot.COUNTS_PER_INCH,false);
//        } else if (nRings == 1) {
//            goToPostion(48 * robot.COUNTS_PER_INCH,115 * robot.COUNTS_PER_INCH,0.8,90,3 * robot.COUNTS_PER_INCH,false);
//        } else {
//            goToPostion(24 * robot.COUNTS_PER_INCH,95 * robot.COUNTS_PER_INCH,0.8,90,3 * robot.COUNTS_PER_INCH,false);
//        }
//
//        //open hand and move elbow to drop wobble goal
//        dropWobbleGoal();
//
//        // turn shooter first to rev up them
//        robot.shooterRight.setPower(0.6);
//        robot.shooterLeft.setPower(-0.6);
//
//        //move to intermediate pos
//        if (nRings == 4 ) {
//            goToPostion(36 * robot.COUNTS_PER_INCH,135 * robot.COUNTS_PER_INCH,0.8,90,3 * robot.COUNTS_PER_INCH,false);
//        } else if (nRings == 1) {
//            goToPostion(60 * robot.COUNTS_PER_INCH,115 * robot.COUNTS_PER_INCH,0.8,90,3 * robot.COUNTS_PER_INCH,false);
//        }
//
//        //Drive to lanch line
//        goToPostion(39 * robot.COUNTS_PER_INCH,69 * robot.COUNTS_PER_INCH,0.8,0,3 * robot.COUNTS_PER_INCH,false);
//
//        //shoot powershot targets
//        rapidFireDisks();
//
//        //park on launch line
//        goToPostion(36 * robot.COUNTS_PER_INCH,84 * robot.COUNTS_PER_INCH,0.8,0,3 * robot.COUNTS_PER_INCH,false);

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

                    // removed capture counter to always use 0 in the filename
                    File file = new File(captureDirectory, String.format(Locale.getDefault(), "VuforiaFrame-%d.png", 0));
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
