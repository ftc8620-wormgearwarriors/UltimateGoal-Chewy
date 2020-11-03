package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.chewy_AutonomousMethods;

public class chewy_CameraTest extends OpMode {
    chewy_HardwareMap robot = new chewy_HardwareMap();
    chewy_AutonomousMethods autoMethods = new chewy_AutonomousMethods();

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        int VuforiaInitOk = autoMethods.VuforiaInit();
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


        if (gamepad2.x)
        {
            //grab picture
        }

    }

    @Override
    public void stop() {
    }

}
