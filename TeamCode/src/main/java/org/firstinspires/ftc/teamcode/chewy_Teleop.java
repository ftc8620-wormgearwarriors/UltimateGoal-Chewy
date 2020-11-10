package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

        double maxVel = 0.5;
        double dropServoPos = 1.1;
        double openServoPos = 0.5;
    @Override
    public void loop() {
        double markerServoPos = .72;
        double frontLeft;
        double backLeft;
        double frontRight;
        double backRight;
        double intakeBottom;
        double intakeMiddle;
        double intakeTop;
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


        telemetry.update();

        // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        frontRight = y_prime - (gamepad1.right_stick_x / 2 * maxVel) + x_prime;
        backRight = y_prime - (gamepad1.right_stick_x / 2 * maxVel) - x_prime;
        frontLeft = y_prime + (gamepad1.right_stick_x / 2 * maxVel) - x_prime;
        backLeft = y_prime + (gamepad1.right_stick_x / 2 * maxVel) + x_prime;

        if (gamepad1.left_trigger > .05)
            maxVel = 0.5;
        else if (gamepad1.right_trigger > .05)
            maxVel = 1.0;
        else if (gamepad1.left_bumper)
            maxVel = 0.25;



        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(backLeft)), Math.max(Math.abs(frontRight), Math.abs(backRight)));
        if (max > 1) {
            frontLeft /= max;
            backLeft /= max;
            frontRight /= max;
            backRight /= max;
        }


        robot.frontLeftDrive.setPower(frontLeft);
        robot.frontRightDrive.setPower(frontRight);
        robot.backLeftDrive.setPower(backLeft);
        robot.backRightDrive.setPower(backRight);



        // stop intake
        if (gamepad2.x)
        {
            robot.intake.setPosition(0);
        }

        // stop firstTransfer
        if (gamepad2.dpad_down)
        {
            robot.firstTransfer.setPosition(0);
        }

        // stop secondTransfer
        if (gamepad2.dpad_right)
        {
            robot.secondTransfer.setPosition(0);
        }

        //intake full power
        if (gamepad2.y)
        {
            robot.intake.setPosition(-1);
        }

        //firstTransfer full power
        if (gamepad2.b)
        {
            robot.firstTransfer.setPosition(-1);
        }

        //secondTransfer full power
        if (gamepad2.a)
        {
            robot.secondTransfer.setPosition(-1);
        }

        // intake half power

//        if (gamepad2.b)
//        {
//            robot.intake.setPosition(-.5);
//            robot.firstTransfer.setPosition(1);
//            robot.secondTransfer.setPosition(1);
//        }


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
        if (gamepad2.right_bumper) {

            robot.wobbleGrabberArm.setPosition(1);
            robot.wobbleGrabberArm.setPosition(1);
            // Find positions for wobble grabber arm and find a button
        }
        if (gamepad2.right_bumper) {

            robot.wobbleGrabberClaw.setPosition(1);
            robot.wobbleGrabberClaw.setPosition(1);
            // Find positions for wobble grabber claw and find a button
        }


    }

    @Override
    public void stop() {
    }

}
