package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

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

@TeleOp(name="chewy_Demo-Teleop", group="Chewy")

public class chewy_Demo_Teleop extends OpMode {
    chewy_HardwareMap robot = new chewy_HardwareMap();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timerShooter = new ElapsedTime();
    ElapsedTime timerIntake = new ElapsedTime();

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
        robot.imu.resetHeading();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    double maxVel = 0.25;
    final double MaxShootTime = 30;

    @Override
    public void loop() {
        //telemetry.addData("Gamepad2 ID", gamepad2.id);
        telemetry.addLine("Intake on   Dpad UP");
        telemetry.addLine("Intake off  Dpad L/R");
        telemetry.addLine("Intake OUT  Dpad DWN");
        telemetry.addLine("Shooter ON  Right Bumper");
        telemetry.addLine("Shooter Off Left Bumper");
        telemetry.addLine("Shooter slw Left Trig");
        telemetry.addLine("Fire Ring!  Right Trig");

        telemetry.update();

        if (gamepad1.x)
            maxVel = 0.25;
        else if (gamepad1.a && !gamepad1.start) // don't change w/ Start-a push
            maxVel = 0.5;
        else if (gamepad1.b && gamepad1.left_stick_button)
            maxVel = 1.0;

        if (gamepad1.y)  {
            robot.imu.resetHeading();
        }

        // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        //  Find robot's current axes in relation to original axes
        double x_axis = -gamepad1.left_stick_x * maxVel;
        double y_axis = -gamepad1.left_stick_y * maxVel;
        double theta = Math.toRadians(-robot.imu.getHeading());
        double x_prime = x_axis * Math.cos(theta) + y_axis * Math.sin(theta);
        double y_prime = -x_axis * Math.sin(theta) + y_axis * Math.cos(theta);

        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        double frontRight  = y_prime - (gamepad1.right_stick_x  * maxVel) + x_prime;
        double backRight   = y_prime - (gamepad1.right_stick_x  * maxVel) - x_prime;
        double frontLeft   = y_prime + (gamepad1.right_stick_x  * maxVel) - x_prime;
        double backLeft    = y_prime + (gamepad1.right_stick_x  * maxVel) + x_prime;

        // Normalize the values so neither exceed +/- 1.0
        double max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(backLeft)), Math.max(Math.abs(frontRight), Math.abs(backRight)));
        if (max > 1) {
            frontLeft /= max;
            backLeft /= max;
            frontRight /= max;
            backRight /= max;
        }
        // now send the calculated values out to the motors
        robot.frontLeftDrive.setPower(frontLeft);
        robot.frontRightDrive.setPower(frontRight);
        robot.backLeftDrive.setPower(backLeft);
        robot.backRightDrive.setPower(backRight);


        //gamepad 2
        boolean NoGP2 = (gamepad2.id  == -1);
        //start all Intake
        if (gamepad2.dpad_up || (NoGP2 && gamepad1.dpad_up))
        {
            robot.intake.setPower(-1);
            timerIntake.reset();
        }

        // stop all Intake
        if (gamepad2.dpad_left || gamepad2.dpad_right || (NoGP2 && (gamepad1.dpad_left || gamepad1.dpad_right)) || timerIntake.seconds()>30.0)
        {
            robot.intake.setPower(0);
        }
        // stop reverse all
        if (gamepad2.dpad_down || (NoGP2 && gamepad1.dpad_down))
        {
            robot.intake.setPower(1);
            timerIntake.reset();
        }

        //pusher or shooter servo
        // if trigger is pushed move servo for at least 1 second.
        // reset the timer to zero when we set shooter servo then don't check
        // again until after 1 second.
        // Additonally don't try to shoot unless spinner motors have been running for at least 1 second
        if (timer.seconds() > 1.0) {
            boolean shooterON;
            if ((robot.shooterRight.getPower() > 0) && (timerShooter.seconds() > 1.0) )
                shooterON = true;
            else
                shooterON = false;
            if ((gamepad2.right_trigger > 0.5 || (NoGP2 && gamepad1.right_trigger > 0.5)) && shooterON) {
                robot.pusher.setPosition(0.65);
                timer.reset();
            } else {
                robot.pusher.setPosition(-1);
            }
        }


        // Button to start the shooter
        if (gamepad2.right_bumper || (NoGP2 && gamepad1.right_bumper))
        {
            robot.shooterRight.setPower(1);
            robot.shooterLeft.setPower(-0.7);
            timerShooter.reset();
        }

        // Button to stop the shooter.
        if (gamepad2.left_bumper || (NoGP2 && gamepad1.left_bumper) || timerShooter.seconds() > MaxShootTime) {
            robot.shooterRight.setPower(0);
            robot.shooterLeft.setPower(0);
        }
        //shooter slow speed
        if (gamepad2.left_trigger > 0.5 || (NoGP2 && gamepad1.left_trigger > 0.5)) {
            robot.shooterRight.setPower(0.8);
            robot.shooterLeft.setPower(-0.65);
            timerShooter.reset();
        }
    }

    @Override
    public void stop() {
    }




        }



