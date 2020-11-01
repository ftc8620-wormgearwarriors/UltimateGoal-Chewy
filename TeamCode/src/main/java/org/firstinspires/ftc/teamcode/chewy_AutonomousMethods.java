package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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
import com.vuforia.CameraDevice;


import java.sql.Time;
import java.util.Timer;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;



public abstract class chewy_AutonomousMethods extends LinearOpMode {    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;

    chewy_HardwareMap robot = new chewy_HardwareMap();

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

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
    private static final String VUFORIA_KEY =
            " --- YOUR NEW VUFORIA KEY GOES HERE  --- ";
    public int VuforiaInint (){
        final String VUFORIA_KEY =
                " Ac/bw0P/////AAABmdRCZF/Kqk2MjbJIs87MKVlJg32ktQ2Tgl6871UmjRacrtxKJCUzDAeC2aA4tbiTjejLjl1W6e7VgBcQfpYx2WhqclKIEkguBRoL1udCrz4OWonoLn/GCA+GntFUZN0Az+dGGYtBqcuW3XkmVNSzgOgJbPDXOf+73P5qb4/mHry0xjx3hysyAzmM/snKvGv8ImhVOVpm00d6ozC8GzvOMRF/S5Z1NBsoFls2/ul+PcZ+veKwgyPFLEFP4DXSqTeOW1nJGH9yYXSH0kfNHgGutLM5om1hAlxdP8D4XMRD2bgWXj1Md2bz+uJmr1E2ZuI7p26ZRxOIKZE9Hwpai+MW6yaJD0otF6aL9QXYaULPpWKo ";
        WebcamName webcamName = null;

        boolean targetVisible = false;
        float phoneXRotate    = 0;
        float phoneYRotate    = 0;
        float phoneZRotate    = 0;

        // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
        // We will define some constants and conversions here
        final float mmPerInch        = 25.4f;
        final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

        // Constants for perimeter targets
        final float halfField = 72 * mmPerInch;
        final float quadField  = 36 * mmPerInch;

        // Class Members
        OpenGLMatrix lastLocation = null;
        VuforiaLocalizer vuforia = null;


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.cameraName = webcamName;

        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);



        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


















        return 0;
    }

//
//        /**
//         * In order for localization to work, we need to tell the system where each target is on the field, and
//         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
//         * Transformation matrices are a central, important concept in the math here involved in localization.
//         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
//         * for detailed information. Commonly, you'll encounter transformation matrices as instances
//         * of the {@link OpenGLMatrix} class.
//         *
//         * If you are standing in the Red Alliance Station looking towards the center of the field,
//         *     - The X axis runs from your left to the right. (positive from the center to the right)
//         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
//         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
//         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
//         *
//         * Before being transformed, each target image is conceptually located at the origin of the field's
//         *  coordinate system (the center of the field), facing up.
//         */
//


//        //
//        // Create a transformation matrix describing where the phone is on the robot.
//        //
//        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
//        // Lock it into Portrait for these numbers to work.
//        //
//        // Info:  The coordinate frame for the robot looks the same as the field.
//        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
//        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
//        //
//        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
//        // pointing to the LEFT side of the Robot.
//        // The two examples below assume that the camera is facing forward out the front of the robot.
//
//        // We need to rotate the camera around it's long axis to bring the correct camera forward.
//        if (CAMERA_CHOICE == BACK) {
//            phoneYRotate = -90;
//        } else {
//            phoneYRotate = 90;
//        }
//
//        // Rotate the phone vertical about the X axis if it's in portrait mode
//        if (PHONE_IS_PORTRAIT) {
//            phoneXRotate = 90 ;
//        }
//
//        // Next, translate the camera lens to where it is on the robot.
//        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
//        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
//        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
//        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
//
//        OpenGLMatrix robotFromCamera = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
//
//        /**  Let all the trackable listeners know where the phone is.  */
//        for (VuforiaTrackable trackable : allTrackables) {
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
//        }
//
//        // WARNING:
//        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
//        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
//        // CONSEQUENTLY do not put any driving commands in this loop.
//        // To restore the normal opmode structure, just un-comment the following line:
//
//        // waitForStart();
//
//        // Note: To use the remote camera preview:
//        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
//        // Tap the preview window to receive a fresh image.
//
//        targetsUltimateGoal.activate();
//        while (!isStopRequested()) {
//
//            // check all the trackable targets to see which one (if any) is visible.
//            targetVisible = false;
//            for (VuforiaTrackable trackable : allTrackables) {
//                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
//                    telemetry.addData("Visible Target", trackable.getName());
//                    targetVisible = true;
//
//                    // getUpdatedRobotLocation() will return null if no new information is available since
//                    // the last time that call was made, or if the trackable is not currently visible.
//                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
//                    if (robotLocationTransform != null) {
//                        lastLocation = robotLocationTransform;
//                    }
//                    break;
//                }
//            }
//
//            // Provide feedback as to where the robot is located (if we know).
//            if (targetVisible) {
//                // express position (translation) of robot in inches.
//                VectorF translation = lastLocation.getTranslation();
//                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//
//                // express the rotation of the robot in degrees.
//                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//            }
//            else {
//                telemetry.addData("Visible Target", "none");
//            }
//            telemetry.update();
//        }
//
//        // Disable Tracking when we are done;
//        targetsUltimateGoal.deactivate();
//    }

    public void Init() {
        robot.init(hardwareMap);

    }

    //Odometry Section
    public void initOdometryHardware(double x, double y, double heading){

        robot.verticalLeft = hardwareMap.dcMotor.get("frontRightDrive");
        robot.verticalRight = hardwareMap.dcMotor.get("backRightDrive");
        robot.horizontal = hardwareMap.dcMotor.get("IntakeRight");


        robot.verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status", "Odometry Init Complete");
        telemetry.update();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        robot.globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, robot.COUNTS_PER_INCH,x * robot.COUNTS_PER_INCH, y* robot.COUNTS_PER_INCH,heading,75); //(135,111) orientation 90
        robot.positionThread = new Thread(robot.globalPositionUpdate);
        robot.positionThread.start();
        robot.globalPositionUpdate.reverseRightEncoder();
        //robot.globalPositionUpdate.reverseNormalEncoder();

    }



    public void  goToPostion(double targetXPostion, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError, boolean pivot) {

        double distanceToXTarget = targetXPostion - robot.globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - robot.globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        double angleError = angleError180(robot.globalPositionUpdate.returnOrientation(),desiredRobotOrientation);
        PIDController           pidRotate;
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
        pidRotate.setInputRange(0,360);
        pidRotate.setOutputRange(0, robotPower);
        pidRotate.setTolerance(1);
        pidRotate.setContinuous(true);
        pidRotate.enable();

        PIDController           pidDrive;

        pidDrive = new PIDController(1/(15 * robot.COUNTS_PER_INCH), 1/(30000 * robot.COUNTS_PER_INCH), 0); //P was 1/60


        pidDrive.reset();
        pidDrive.setSetpoint(targetYPosition);
        pidDrive.setInputRange(0,144 * robot.COUNTS_PER_INCH);
        pidDrive.setOutputRange(0, robotPower);
        pidDrive.setTolerance(1);
        pidDrive.setContinuous(false);
        pidDrive.enable();

        PIDController           pidStrafe;

        pidStrafe = new PIDController(1/(8 * robot.COUNTS_PER_INCH), 1/(2000 * robot.COUNTS_PER_INCH), 0);


        pidStrafe.reset();
        pidStrafe.setSetpoint(targetXPostion);
        pidStrafe.setInputRange(0,144 * robot.COUNTS_PER_INCH);
        pidStrafe.setOutputRange(0, robotPower);
        pidStrafe.setTolerance(1);
        pidStrafe.setContinuous(false);
        pidStrafe.enable();

        double pivotCorrection = pidRotate.performPID(robot.globalPositionUpdate.returnOrientation()); // power will be - on right turn.

        double startTime = getRuntime();


        while (opModeIsActive()&& (distance > allowableDistanceError || !pidRotate.onTarget())) {
            if (!robot.globalPositionUpdate.getNewData())
                continue;

            angleError = angleError180(robot.globalPositionUpdate.returnOrientation(),desiredRobotOrientation);

            distanceToXTarget = targetXPostion - robot.globalPositionUpdate.returnXCoordinate();  // distance to X target
            distanceToYTarget = targetYPosition - robot.globalPositionUpdate.returnYCoordinate(); // distance to Y target

            distance = Math.hypot(distanceToXTarget,distanceToYTarget); // calculate offset distance

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget)) - robot.globalPositionUpdate.returnOrientation(); // angle robot is moving

            if (distance > allowableDistanceError * 2) {
                startTime = getRuntime();
            }
            if ((pivot == false) && (getRuntime() - startTime > 1))  {
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
            double robot_movement_y_component = pidDrive.performPID(targetYPosition   - cY);
            if (pivot) {
                robot_movement_x_component = 0;
                robot_movement_y_component = 0;
            }

            pivotCorrection = pidRotate.performPID(robot.globalPositionUpdate.returnOrientation()); // power will be - on right turn.

            double frontLeftPower =  robot_movement_y_component + robot_movement_x_component + pivotCorrection;
            double frontRightPower = robot_movement_y_component - robot_movement_x_component - pivotCorrection;
            double backLeftPower =   robot_movement_y_component - robot_movement_x_component + pivotCorrection;
            double backRightPower =  robot_movement_y_component + robot_movement_x_component - pivotCorrection;

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
                    "  x = " +robot.globalPositionUpdate.returnXCoordinate()    / robot.COUNTS_PER_INCH +
                    "  y = " + robot.globalPositionUpdate.returnYCoordinate()   / robot.COUNTS_PER_INCH +
                    "  angle ="+ robot.globalPositionUpdate.returnOrientation() +
                    "  angle_error ="+pidRotate.getError() +
                    "  Y_error ="+ distanceToYTarget    / robot.COUNTS_PER_INCH +
                    "  X_error =" + distanceToXTarget     / robot.COUNTS_PER_INCH +
                    "  distance Error =" + distance       / robot.COUNTS_PER_INCH +
                    "  cX ="+cX +
                    "  cY ="+ cY );
        }
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0) ;
        robot.backRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0) ;
    }



    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
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

}


