package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="chewy_Teleop", group="Pushbot")

public class chewy_Teleop extends OpMode {
    chewy_HardwareMap robot = new chewy_HardwareMap();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {


    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    double wobbleGrabberUpDownPos = 0.5;
    double wobbleGrabberOpenClosePos = 0.5;
    double wobbleGrabberUpDownMinPos = 0.35;
    double wobbleGrabberUpDownMaxPos = 0.55;
    double wobbleGrabberOpenCloseMinPos = 0.0;
    double wobbleGrabberOpenCloseMaxPos = 1.0;

    double maxVel = 0.5;
    boolean autoCollect = false;
    @Override
    public void loop() {
        double markerServoPos = .72;
        double frontLeft;
        double backLeft;
        double frontRight;
        double backRight;
        double max;
        double x_axis = -gamepad1.left_stick_x * maxVel;
        double y_axis = -gamepad1.left_stick_y * maxVel;
        double x_prime;
        double y_prime;
        double theta = Math.toRadians(-robot.imu.getHeading());
        double gyroHeading = robot.imu.getHeading();


        //  Find robot's current axes in relation to original axes
        x_prime = x_axis * Math.cos(theta) + y_axis * Math.sin(theta);
        y_prime = -x_axis * Math.sin(theta) + y_axis * Math.cos(theta);

        telemetry.addData("Gamepad2.leftstick_x", gamepad2.left_stick_x);
        telemetry.addData("Theta (in radians)", theta);
        telemetry.addData("x_axis", x_axis);
        telemetry.addData("y_axis", y_axis);
        telemetry.addData("x_prime", x_prime);
        telemetry.addData("y_prime", y_prime);
        telemetry.addData("Gyro Heading", gyroHeading);
        telemetry.addData("wobbleGrabberUpDown", String.format ("%.05f", wobbleGrabberUpDownPos));
        telemetry.addData("wobbleGrabberOpenClose", String.format ("%.05f", wobbleGrabberOpenClosePos));
        telemetry.addData("Left Range", robot.leftRange.cmUltrasonic());
        telemetry.addData("front range", String.format("%.01f cm", robot.frontRange.getDistance(DistanceUnit.CM)));

        telemetry.update();

        // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        frontRight = y_prime - (gamepad1.right_stick_x  * maxVel) + x_prime;
        backRight = y_prime - (gamepad1.right_stick_x  * maxVel) - x_prime;
        frontLeft = y_prime + (gamepad1.right_stick_x  * maxVel) - x_prime;
        backLeft = y_prime + (gamepad1.right_stick_x  * maxVel) + x_prime;

        if (gamepad1.left_trigger > .05)
            maxVel = 0.5;
        else if (gamepad1.right_trigger > .05)
            maxVel = 1.0;
        else if (gamepad1.left_bumper)
            maxVel = 0.25;

        if (gamepad1.y)  {
            robot.imu.resetHeading();
        }

//        if (gamepad1.dpad_down) {
//            if (wobbleGrabberUpDownPos > wobbleGrabberUpDownMinPos) {
//                wobbleGrabberUpDownPos -= 0.001;
//
//            }
//        }
//        if (gamepad1.dpad_up) {
//            if (wobbleGrabberUpDownPos < wobbleGrabberUpDownMaxPos) {
//                wobbleGrabberUpDownPos += 0.001;
//            }
//        }
//        robot.wobbleGrabberUpDown.setPosition(wobbleGrabberUpDownPos);
//
//        if (gamepad1.dpad_right) {
//            if (wobbleGrabberOpenClosePos > wobbleGrabberOpenCloseMinPos) {
//                wobbleGrabberOpenClosePos -= 0.05;
//
//            }
//        }
//        if (gamepad1.dpad_left) {
//            if (wobbleGrabberOpenClosePos < wobbleGrabberOpenCloseMaxPos) {
//                wobbleGrabberOpenClosePos += 0.05;
//            }
//        }
//        robot.wobbleGrabberOpenClose.setPosition(wobbleGrabberOpenClosePos);



        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(backLeft)), Math.max(Math.abs(frontRight), Math.abs(backRight)));
        if (max > 1) {
            frontLeft /= max;
            backLeft /= max;
            frontRight /= max;
            backRight /= max;
        }

        if (gamepad1.right_bumper) {    //high goal
            shootingSpot (183,1,61);
        } else if (gamepad1.x) {        //left power shot
            shootingSpot(150,0.5,120);
        } else if (gamepad1.a) {        //middle power shot
            shootingSpot(150,0.5,127);
        } else if (gamepad1.b){         //right power shot
            shootingSpot(150,0.5,149.86);
        } else {                       //manual
            robot.frontLeftDrive.setPower(frontLeft);
            robot.frontRightDrive.setPower(frontRight);
            robot.backLeftDrive.setPower(backLeft);
            robot.backRightDrive.setPower(backRight);
        }


        //gamepad 2

        //start all
        if (gamepad2.x)
        {
            robot.intake.setPower(1);
            robot.intakeRoller.setPosition(1);
            robot.firstTransfer.setPosition(1);
            robot.secondTransfer.setPosition(1);
        }

        // stop all
        if (gamepad2.y)
        {
            robot.intake.setPower(0);
            robot.intakeRoller.setPosition(0.5);
            robot.firstTransfer.setPosition(0.5);
            robot.secondTransfer.setPosition(0.5);
        }

        //pusher servo
        if(gamepad2.right_trigger > 0.5)
        {
            robot.pusher.setPosition(1);
        }
        else{
            robot.pusher.setPosition(-1);
        }




        // stop reverse all
        if (gamepad2.dpad_up)
        {
            robot.secondTransfer.setPosition(0);
            robot.firstTransfer.setPosition(0);
            robot.intake.setPower(-1);
        }

        // Button to start the shooter
        if (gamepad2.left_bumper)
        {
            robot.shooterRight.setPower(1);
            robot.shooterLeft.setPower(-0.7);
        }

        // Button to stop the shooter.
        if (gamepad2.right_bumper) {

            robot.shooterRight.setPower(0);
            robot.shooterLeft.setPower(0);
        }

        //shooter slow speed
        if (gamepad2.dpad_down) {

            robot.shooterRight.setPower(0.5);
            robot.shooterLeft.setPower(-0.55);
        }

        //wobble grabber controls
        if (gamepad2.left_stick_y > 0.5) {
            if (wobbleGrabberUpDownPos > wobbleGrabberUpDownMinPos) {
                wobbleGrabberUpDownPos -= 0.001;

            }
        }
        if (gamepad2.left_stick_y < -0.5) {
            if (wobbleGrabberUpDownPos < wobbleGrabberUpDownMaxPos) {
                wobbleGrabberUpDownPos += 0.001;
            }
        }
        robot.wobbleGrabberUpDown.setPosition(wobbleGrabberUpDownPos);

        if (gamepad2.left_stick_x < -0.5) {
            if (wobbleGrabberOpenClosePos > wobbleGrabberOpenCloseMinPos) {
                wobbleGrabberOpenClosePos -= 0.05;

            }
        }
        if (gamepad2.left_stick_x > 0.5) {
            if (wobbleGrabberOpenClosePos < wobbleGrabberOpenCloseMaxPos) {
                wobbleGrabberOpenClosePos += 0.05;
            }
        }
        robot.wobbleGrabberOpenClose.setPosition(wobbleGrabberOpenClosePos);


        }

    @Override
    public void stop() {
    }


    //line up to shoot
    double  minVel              = 0.1;
    double  vel                 = minVel;
    double  oldVel              = minVel;
    public double shootingSpot (double distance, double maxVel, double gapDistance) {
        double  targetHeading       = 90;
        double  kpTurn              = 0.05;
        double  kpDistance          = 0.01;
        double  kpGap               = 0.01; //was 0.03
        double  accel               = 0.03;







        double distanceError = robot.frontRange.getDistance(DistanceUnit.CM) - distance;


        vel = Math.abs(distanceError * kpDistance);



        if (vel > (oldVel + accel))

            vel = oldVel + accel;

        if (vel > (Math.abs(maxVel))) {
            vel = (Math.abs(maxVel));
        }

        if (vel < minVel) {
            vel = minVel;

        }
        oldVel = vel;

        if(distanceError < 0)
            vel = -vel;

        double error = -angleErrorDrive(targetHeading, robot.imu.getHeading());

        double sideErr;
//            if (side == sensorSide.RIGHT)
//                gap_err = robot.rightRangeSensor.cmUltrasonic() - gapDistance;
//            else
//                gap_err = - (robot.leftRangeSensor.cmUltrasonic() - gapDistance);
        sideErr = - (robot.leftRange.cmUltrasonic() - gapDistance);

        //tells us that when we sense another robot to change nothing
//            if (Math.abs(sideErr)>30)
//                sideErr = 0;


        // Set motors to specified power
        double  frontLeftPower      = vel - (error * kpTurn)  + (sideErr * kpGap);
        double  frontRightPower     = vel + (error * kpTurn)  - (sideErr * kpGap);
        double  backLeftPower       = vel - (error * kpTurn)  - (sideErr * kpGap);
        double  backRightPower      = vel + (error * kpTurn)  + (sideErr * kpGap);

        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));
        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        robot.frontLeftDrive.setPower (frontLeftPower);
        robot.frontRightDrive.setPower(frontRightPower);
        robot.backLeftDrive.setPower  (backLeftPower);
        robot.backRightDrive.setPower (backRightPower);

//        telemetry.addData("front right: ", robot.frontRightDrive.getCurrentPosition());
//        telemetry.addData("front left:", robot.frontLeftDrive.getCurrentPosition());
//        telemetry.addData("back right:", robot.backRightDrive.getCurrentPosition());
//        telemetry.addData("back left:", robot.backLeftDrive.getCurrentPosition());
//        telemetry.addData("FR Speed:", robot.frontRightDrive.getPower());
//        telemetry.addData("FL Speed:", robot.frontLeftDrive.getPower());
//        telemetry.addData("BR Speed", robot.backRightDrive.getPower());
//        telemetry.addData("BL Speed:", robot.backLeftDrive.getPower());
        telemetry.addData("Left Range", robot.leftRange.cmUltrasonic());
        telemetry.addData("front range", String.format("%.01f cm", robot.frontRange.getDistance(DistanceUnit.CM)));
        telemetry.addData("turn error", error);
        telemetry.addData("gyro ", robot.imu.getHeading());
        //telemetry.addData("left GAP", robot.leftRangeSensor.cmUltrasonic());
        telemetry.update();
        RobotLog.d("8620WGW shootingSpot " +
                "gyro " + robot.imu.getHeading() + ", " +
                "left error " + sideErr + ", " +
                "front error " + distanceError + ", " +
                "turn error " + error + ", ");


        // }

        // Turn off motors
//        robot.frontLeftDrive    .setPower(0);
//        robot.frontRightDrive   .setPower(0);
//        robot.backLeftDrive     .setPower(0);
//        robot.backRightDrive    .setPower(0);

        // Return average total ticks traveled
        return 0;
    }   // gap()

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


        }



