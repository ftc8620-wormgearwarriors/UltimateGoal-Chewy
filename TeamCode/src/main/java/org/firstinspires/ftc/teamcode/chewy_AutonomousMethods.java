package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.CameraDevice;


import java.sql.Time;
import java.util.Locale;
import java.util.Timer;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.vuforia.Frame;


public class chewy_AutonomousMethods extends LinearOpMode {    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;


    @Override
    // need this to extend LinearOpMode
    public void runOpMode() {


    }


    chewy_HardwareMap robot = new chewy_HardwareMap();

    private VuforiaLocalizer vuforia = null;
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    public static final String TAG = "Vuforia Navigation Sample";
    int captureCounter = 0;
//    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
//    private static final boolean PHONE_IS_PORTRAIT = false;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */

    // private GoldAlignDetector detector;

    public enum sensorSide {
        LEFT,
        RIGHT
    }

    public enum sensorFront {
        DEATHSTAR,
        WOOKIE,                         //upper ultrasonic sensor
        NOSENSOR
    }

    public void Init() {
        robot.init(hardwareMap);

    }

    public void motorTest() {
        /* silly test of wheel motors */
        //Restart tick count from encoders
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("test Front Left");
        telemetry.update();
        while (robot.frontLeftDrive.getCurrentPosition() < 100)
            robot.frontLeftDrive.setPower(0.1);
        robot.frontLeftDrive.setPower(0);

        telemetry.addLine("test Front Right");
        telemetry.update();
        while (robot.frontRightDrive.getCurrentPosition() < 100)
            robot.frontRightDrive.setPower(0.1);
        robot.frontRightDrive.setPower(0);

        telemetry.addLine("test Back Left");
        telemetry.update();
        while (robot.backLeftDrive.getCurrentPosition() < 100)
            robot.backLeftDrive.setPower(0.1);
        robot.backLeftDrive.setPower(0);

        telemetry.addLine("test Back Right");
        telemetry.update();
        while (robot.backRightDrive.getCurrentPosition() < 100)
            robot.backRightDrive.setPower(0.1);
        robot.backRightDrive.setPower(0);

    }

    /**
     * 0
     * Function to drive straight forward or backward
     * Pass:
     * distance = distance to travel (cm)
     * velocity = motor power (+ value moves forward, - value moves backward)
     * Constants:
     * diameter = wheel diameter
     * circumference = wheel circumference
     * gearRatio = gear ratio from motor to wheel
     * ticksPerRotation = ticks returned from encoder per rotation of motor
     * Variables:
     * ticks = number of ticks to move given distance
     * Return average motor ticks at end of movement
     */
    public double drive(double distance, double maxVel) {
        double diameter = 10.16;
        double circumference = diameter * Math.PI;
        double gearRatio = 20.36;
        int ticksPerRotation = 28;
        double targetTicks = distance * (1 / circumference) * gearRatio * ticksPerRotation;
        double targetHeading = robot.imu.getHeading();
        double kpTurn = 0.01;
        double kpDistance = 0.001; //was 0.0003
        double minVel = 0.1;
        double accel = 0.03;    //0.01;
        double vel = minVel;
        double oldVel = minVel;

        //Restart tick count from encoders
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Loop until average motor ticks reaches specified number of ticks
        while (opModeIsActive() && targetTicks > currentPositionAverage()) {

            double distanceError = targetTicks - currentPositionAverage();
            vel = distanceError * kpDistance;


            if (vel > (oldVel + accel))

                vel = oldVel + accel;

            if (vel > (Math.abs(maxVel))) {
                vel = (Math.abs(maxVel));
            }

            if (vel < minVel) {
                vel = minVel;

            }
            oldVel = vel;

            if (maxVel < 0)
                vel = -vel;

            double error = angleErrorDrive(targetHeading, robot.imu.getHeading());
            // Run motors at specified power
            robot.frontLeftDrive.setPower(vel - (error * kpTurn));
            robot.frontRightDrive.setPower(vel + (error * kpTurn));
            robot.backLeftDrive.setPower(vel - (error * kpTurn));
            robot.backRightDrive.setPower(vel + (error * kpTurn));

            telemetry.addData("front right: ", robot.frontRightDrive.getCurrentPosition());
            telemetry.addData("front left:", robot.frontLeftDrive.getCurrentPosition());
            telemetry.addData("back right:", robot.backRightDrive.getCurrentPosition());
            telemetry.addData("back left:", robot.backLeftDrive.getCurrentPosition());
            telemetry.addData("FR Speed:", robot.frontRightDrive.getPower());
            telemetry.addData("FL Speed:", robot.frontLeftDrive.getPower());
            telemetry.addData("BR Speed", robot.backRightDrive.getPower());
            telemetry.addData("BL Speed:", robot.backLeftDrive.getPower());
            telemetry.update();
        }

        // Turn off motors
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        robot.backRightDrive.setPower(0);

        // Return average total ticks traveled
        return currentPositionAverage();
    }

    public double currentPositionAverage() {
        return ((Math.abs(robot.frontLeftDrive.getCurrentPosition()) +
                Math.abs(robot.frontRightDrive.getCurrentPosition()) +
                Math.abs(robot.backLeftDrive.getCurrentPosition()) +
                Math.abs(robot.backRightDrive.getCurrentPosition())) / 4);

    }

    /**
     * Created by Worm Gear Warriors on 10/28/2018.
     * returns an angle between -180 and +180
     */
    public double angleErrorDrive(double angleTarget, double angleInitial) {
        double error = angleInitial - angleTarget;

        while (error <= -180 || error > 180) {
            if (error > 180) {
                error = error - 360;
            }
            if (error <= -180) {
                error = error + 360;
            }
        }
        return error;
    }

    /**
     * Created by Worm Gear Warriors on 10/28/2018.
     * returns an angle between 0 and 360
     */
    public double angleErrorRotate(double angleTarget, double angleInitial) {
        double error = angleInitial - angleTarget;

        while (error < 0 || error >= 360) {
            if (error >= 360) {
                error = error - 360;
            }
            if (error < 0) {
                error = error + 360;
            }
        }
        return error;
    }

    /**
     * Function to drive straight forward or backward
     * Pass:
     * distance = distance to travel (cm)
     * velocity = motor power (+ value moves forward, - value moves backward)
     * Constants:
     * diameter = wheel diameter
     * circumference = wheel circumference
     * gearRatio = gear ratio from motor to wheel
     * ticksPerRotation = ticks returned from encoder per rotation of motor
     * Variables:
     * ticks = number of ticks to move given distance
     * Return average motor ticks at end of movement
     */
    public double strafe(double distance, double maxVel, double frontDistance, sensorFront Front) {
        double diameter = 10.16;
        double circumference = diameter * Math.PI;
        double gearRatio = 20.36;
        int ticksPerRotation = 28;
        double targetTicks = distance * (1 / circumference) * gearRatio * ticksPerRotation;
        double targetHeading = robot.imu.getHeading();
        double kpTurn = 0.01;
        double kpDistance = 0.01;
        double kpFrontDistance = 0.01;
        double minVel = 0.1;
        double accel = 0.03;      //0.01;
        double vel = minVel;
        double oldVel = minVel;
        double frontDistanceError = 0;

        //Restart tick count from encoders
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Loop until average motor ticks reaches specified number of ticks
        while (opModeIsActive() && targetTicks > currentPositionAverage()) {

            double distanceError = targetTicks - currentPositionAverage();
            vel = distanceError * kpDistance;


            if (vel > (oldVel + accel))

                vel = oldVel + accel;

            if (vel > (Math.abs(maxVel))) {
                vel = (Math.abs(maxVel));
            }

            if (vel < minVel) {
                vel = minVel;

            }
            oldVel = vel;

            if (maxVel < 0)
                vel = -vel;

            double error = angleErrorDrive(targetHeading, robot.imu.getHeading());

            //decides to use frontDistance sensor
//            if (Front == sensorFront.DEATHSTAR)
//                frontDistanceError = robot.deathStar.getDistance(DistanceUnit.CM) - frontDistance;
//            else if (Front == sensorFront.WOOKIE)
//                frontDistanceError = robot.wookie.getDistance(DistanceUnit.CM) - frontDistance;
//            else
//                frontDistanceError = 0;

            // Run motors at specified power
            robot.frontLeftDrive.setPower(-vel - (error * kpTurn) + (frontDistanceError * kpFrontDistance));
            robot.frontRightDrive.setPower(vel + (error * kpTurn) + (frontDistanceError * kpFrontDistance));
            robot.backLeftDrive.setPower(vel - (error * kpTurn) + (frontDistanceError * kpFrontDistance));
            robot.backRightDrive.setPower(-vel + (error * kpTurn) + (frontDistanceError * kpFrontDistance));

            telemetry.addData("front right:", robot.frontRightDrive.getCurrentPosition());
            telemetry.addData("front left:", robot.frontLeftDrive.getCurrentPosition());
            telemetry.addData("back right:", robot.backRightDrive.getCurrentPosition());
            telemetry.addData("back left:", robot.backLeftDrive.getCurrentPosition());
            telemetry.addData("FR Speed:", robot.frontRightDrive.getPower());
            telemetry.addData("FL Speed:", robot.frontLeftDrive.getPower());
            telemetry.addData("BR Speed", robot.backRightDrive.getPower());
            telemetry.addData("BL Speed:", robot.backLeftDrive.getPower());
            telemetry.update();
        }

        // Turn off motors
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        robot.backRightDrive.setPower(0);

        // Return average total ticks traveled
        return currentPositionAverage();
    }

    /**
     * Function to drive straight forward or backward
     * Pass:
     * distance = distance to travel (cm)
     * velocity = motor power (+ value moves forward, - value moves backward)
     * Constants:
     * diameter = wheel diameter
     * circumference = wheel circumference
     * gearRatio = gear ratio from motor to wheel
     * ticksPerRotation = ticks returned from encoder per rotation of motor
     * Variables:
     * ticks = number of ticks to move given distance
     * Return average motor ticks at end of movement
     */
    public double rotate(double targetHeading, double maxVel) {

        double kpRotate = 0.015;                        //0.01;
        double minVel = 0.1;
        double accel = 0.02;       //0.01;
        double vel = minVel;
        double oldVel = minVel;
        double headingError = 500;  // force into while loop first time through
        double lastHeadingError = 500;
        boolean done = false;

        //Restart tick count from encoders
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Loop until average motor ticks reaches specified number of ticks
        while (opModeIsActive() && !done) {

            // find how far we are off our desired heading
            // if MaxVel is neagitve we are turning counter-clockwise
            if (maxVel >= 0)
                headingError = angleErrorRotate(robot.imu.getHeading(), targetHeading);
            else
                headingError = angleErrorRotate(targetHeading, robot.imu.getHeading());
            if (headingError > lastHeadingError + 10)
                done = true;
            lastHeadingError = headingError;

            telemetry.addData("Rotateing", headingError);
            telemetry.update();

            vel = headingError * kpRotate;


            if (vel > (oldVel + accel))

                vel = oldVel + accel;

            if (vel > (Math.abs(maxVel))) {
                vel = (Math.abs(maxVel));
            }

            if (vel < minVel) {
                vel = minVel;

            }
            oldVel = vel;

            if (maxVel < 0)
                vel = -vel;

            double error = angleErrorDrive(targetHeading, robot.imu.getHeading());
            // Run motors at specified power
            robot.frontLeftDrive.setPower(vel);
            robot.frontRightDrive.setPower(-vel);
            robot.backLeftDrive.setPower(vel);
            robot.backRightDrive.setPower(-vel);

            telemetry.addData("front right:", robot.frontRightDrive.getCurrentPosition());
            telemetry.addData("front left:", robot.frontLeftDrive.getCurrentPosition());
            telemetry.addData("back right:", robot.backRightDrive.getCurrentPosition());
            telemetry.addData("back left:", robot.backLeftDrive.getCurrentPosition());
            telemetry.addData("FR Speed:", robot.frontRightDrive.getPower());
            telemetry.addData("FL Speed:", robot.frontLeftDrive.getPower());
            telemetry.addData("BR Speed", robot.backRightDrive.getPower());
            telemetry.addData("BL Speed:", robot.backLeftDrive.getPower());
            telemetry.update();
        }

        // Turn off motors
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        robot.backRightDrive.setPower(0);

        // Return average total ticks traveled
        return currentPositionAverage();
    }

    final String VUFORIA_KEY =
            " Ac/bw0P/////AAABmdRCZF/Kqk2MjbJIs87MKVlJg32ktQ2Tgl6871UmjRacrtxKJCUzDAeC2aA4tbiTjejLjl1W6e7VgBcQfpYx2WhqclKIEkguBRoL1udCrz4OWonoLn/GCA+GntFUZN0Az+dGGYtBqcuW3XkmVNSzgOgJbPDXOf+73P5qb4/mHry0xjx3hysyAzmM/snKvGv8ImhVOVpm00d6ozC8GzvOMRF/S5Z1NBsoFls2/ul+PcZ+veKwgyPFLEFP4DXSqTeOW1nJGH9yYXSH0kfNHgGutLM5om1hAlxdP8D4XMRD2bgWXj1Md2bz+uJmr1E2ZuI7p26ZRxOIKZE9Hwpai+MW6yaJD0otF6aL9QXYaULPpWKo ";

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

    //initializes object detector (tfod)
    void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    void captureFrameToFile() {
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>()
        {
            @Override public void accept(Frame frame)
            {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
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

    //use vuforia to capture an image that determines the number of donuts
    int numberOfDonuts(){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
                telemetry.update();
            }
        }

        return 0;
    }





    //Odometry Section
    public void initOdometryHardware(double x, double y, double heading) {

//        robot.verticalLeft = hardwareMap.dcMotor.get("frontRightDrive");
//        robot.verticalRight = hardwareMap.dcMotor.get("backRightDrive");
//        robot.horizontal = hardwareMap.dcMotor.get("IntakeRight");


        robot.verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status", "Odometry Init Complete");
        telemetry.update();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        robot.globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, robot.COUNTS_PER_INCH, x * robot.COUNTS_PER_INCH, y * robot.COUNTS_PER_INCH, heading, 75); //(135,111) orientation 90
        robot.positionThread = new Thread(robot.globalPositionUpdate);
        robot.positionThread.start();
        robot.globalPositionUpdate.reverseLeftEncoder();
        robot.globalPositionUpdate.reverseRightEncoder();
        //robot.globalPositionUpdate.reverseNormalEncoder();

    }


    public void goToPostion(double targetXPostion, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError, boolean pivot) {

        double distanceToXTarget = targetXPostion - robot.globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - robot.globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        double angleError = angleError180(robot.globalPositionUpdate.returnOrientation(), desiredRobotOrientation);
        PIDController pidRotate;
        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.03, .00003, 0); // Kp: .003 then 0.022
        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.


        pidRotate.reset();
        pidRotate.setSetpoint(desiredRobotOrientation);
        pidRotate.setInputRange(0, 360);
        pidRotate.setOutputRange(0, robotPower);
        pidRotate.setTolerance(1);
        pidRotate.setContinuous(true);
        pidRotate.enable();

        PIDController pidDrive;

        pidDrive = new PIDController(1 / (15 * robot.COUNTS_PER_INCH), 1 / (30000 * robot.COUNTS_PER_INCH), 0); //P was 1/60


        pidDrive.reset();
        pidDrive.setSetpoint(targetYPosition);
        pidDrive.setInputRange(0, 144 * robot.COUNTS_PER_INCH);
        pidDrive.setOutputRange(0, robotPower);
        pidDrive.setTolerance(1);
        pidDrive.setContinuous(false);
        pidDrive.enable();

        PIDController pidStrafe;

        pidStrafe = new PIDController(1 / (8 * robot.COUNTS_PER_INCH), 1 / (2000 * robot.COUNTS_PER_INCH), 0);


        pidStrafe.reset();
        pidStrafe.setSetpoint(targetXPostion);
        pidStrafe.setInputRange(0, 144 * robot.COUNTS_PER_INCH);
        pidStrafe.setOutputRange(0, robotPower);
        pidStrafe.setTolerance(1);
        pidStrafe.setContinuous(false);
        pidStrafe.enable();

        double pivotCorrection = pidRotate.performPID(robot.globalPositionUpdate.returnOrientation()); // power will be - on right turn.

        double startTime = getRuntime();


        while (opModeIsActive() && (distance > allowableDistanceError || !pidRotate.onTarget())) {
            if (!robot.globalPositionUpdate.getNewData())
                continue;

            angleError = angleError180(robot.globalPositionUpdate.returnOrientation(), desiredRobotOrientation);

            distanceToXTarget = targetXPostion - robot.globalPositionUpdate.returnXCoordinate();  // distance to X target
            distanceToYTarget = targetYPosition - robot.globalPositionUpdate.returnYCoordinate(); // distance to Y target

            distance = Math.hypot(distanceToXTarget, distanceToYTarget); // calculate offset distance

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget)) - robot.globalPositionUpdate.returnOrientation(); // angle robot is moving

            if (distance > allowableDistanceError * 2) {
                startTime = getRuntime();
            }
            if ((pivot == false) && (getRuntime() - startTime > 1)) {
                telemetry.addLine("Breaked While Loop");
                telemetry.update();
                break;
            }


            //double robot_movement_x_component = calculateX(robotMovementAngle, robotPower * 1.5); // calcuate how much strafe and drive needed to get to target
            //double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            //double pivotCorrection = (desiredRobotOrientation - globalPositionUpdate.returnOrientation()) / 20; // keep robot facing right way
            //double robot_movement_x_component = pidStrafe.performPID(globalPositionUpdate.returnXCoordinate());
            //double robot_movement_y_component = pidDrive.performPID(globalPositionUpdate.returnYCoordinate());
            double cX = calculateX(robotMovementAngle, distance);
            double cY = calculateY(robotMovementAngle, distance);
            double robot_movement_x_component = pidStrafe.performPID(targetXPostion - cX);
            double robot_movement_y_component = pidDrive.performPID(targetYPosition - cY);
            if (pivot) {
                robot_movement_x_component = 0;
                robot_movement_y_component = 0;
            }

            pivotCorrection = pidRotate.performPID(robot.globalPositionUpdate.returnOrientation()); // power will be - on right turn.

            double frontLeftPower = robot_movement_y_component + robot_movement_x_component + pivotCorrection;
            double frontRightPower = robot_movement_y_component - robot_movement_x_component - pivotCorrection;
            double backLeftPower = robot_movement_y_component - robot_movement_x_component + pivotCorrection;
            double backRightPower = robot_movement_y_component + robot_movement_x_component - pivotCorrection;

            double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                    Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));
            if (max > robotPower) {
                double divider = max / robotPower;
                frontLeftPower /= divider;
                frontRightPower /= divider;
                backLeftPower /= divider;
                backRightPower /= divider;
            }
            robot.frontRightDrive.setPower(frontRightPower);
            robot.backRightDrive.setPower(backRightPower);
            robot.frontLeftDrive.setPower(frontLeftPower);
            robot.backLeftDrive.setPower(backLeftPower);
            RobotLog.d("8620WGW goToPosition " +
                    "  x = " + robot.globalPositionUpdate.returnXCoordinate() / robot.COUNTS_PER_INCH +
                    "  y = " + robot.globalPositionUpdate.returnYCoordinate() / robot.COUNTS_PER_INCH +
                    "  angle =" + robot.globalPositionUpdate.returnOrientation() +
                    "  angle_error =" + pidRotate.getError() +
                    "  Y_error =" + distanceToYTarget / robot.COUNTS_PER_INCH +
                    "  X_error =" + distanceToXTarget / robot.COUNTS_PER_INCH +
                    "  distance Error =" + distance / robot.COUNTS_PER_INCH +
                    "  cX =" + cX +
                    "  cY =" + cY);
        }
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
        robot.backRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
    }


    /**
     * Calculate the power in the x direction
     *
     * @param desiredAngle angle on the x axis
     * @param speed        robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     *
     * @param desiredAngle angle on the y axis
     * @param speed        robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Created by Worm Gear Warriors on 10/28/2018.
     * returns an angle between -180 and +180
     */
    public double angleError180(double angleTarget, double angleInitial) {
        double error = angleInitial - angleTarget;

        while (error <= -180 || error > 180) {
            if (error > 180) {
                error = error - 360;
            }
            if (error <= -180) {
                error = error + 360;
            }
        }
        return error;
    }

    //shoot first disk
    public void firstDiskAuto() {
        robot.shooterRight.setPower(.5);
        robot.shooterLeft.setPower(-0.5);
        sleep (3000);
        robot.secondTransfer.setPosition(1);
        sleep (1000);
    }

    //shoot second disk
    public void secondDiskAuto(){
        robot.shooterRight.setPower(.5);
        robot.shooterLeft.setPower(-0.5);
        robot.secondTransfer.setPosition(1);
        robot.firstTransfer.setPosition(1);
        sleep (2000);
    }
    //shoot third disk
    public void thirdDiskAuto(){
        robot.shooterRight.setPower(.5);
        robot.shooterLeft.setPower(-0.5);
        robot.secondTransfer.setPosition(1);
        robot.firstTransfer.setPosition(1);
        robot.intake.setPower(1);
        sleep (8000);
    }
}

