package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="motorTest", group="Pushbot")
@Disabled
public class motorTest extends OpMode {
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
    @Override
    public void loop() {
        double markerServoPos = .72;
        double frontLeft;
        double backLeft;
        double frontRight;
        double backRight;



        telemetry.update();


        if (gamepad1.y) {
            robot.frontLeftDrive.setPower(0.2);
        }
        else {
            robot.frontLeftDrive.setPower(0);
        }


        if (gamepad1.b) {
            robot.frontRightDrive.setPower(0.2);
        }
        else {
            robot.frontRightDrive.setPower(0);
        }


        if (gamepad1.x) {
            robot.backLeftDrive.setPower(0.2);
        }
        else {
            robot.backLeftDrive.setPower(0);
        }


        if (gamepad1.a) {
            robot.backRightDrive.setPower(0.2);
        }
        else {
            robot.backRightDrive.setPower(0);
        }





    }



    @Override
    public void stop() {
    }
}


