package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name = "chewy_Camera_Test")
public class chewy_Camera_Test extends chewy_AutonomousMethods {

    private VuforiaLocalizer vuforia = null;

    @Override
    public void runOpMode() {
        //Initialize hardware map values

        //initializing odometry hardware
        Init();
        initOdometryHardware(55, 9, 0);

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
        ringDetector.setCropBox(0, 160, 150, 310);


        // do method based on counting "yellow pixels"
        int nRings = 0;

        //use the ring detector to find the number of rings
        nRings = ringDetector.getNumberOfRings();

        while (opModeIsActive()) {
            nRings = ringDetector.getNumberOfRings();
            telemetry.addData("RunOpMode:NumRings", nRings);
            telemetry.update();
        }

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