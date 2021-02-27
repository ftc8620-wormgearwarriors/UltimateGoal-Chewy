package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


/**
 * Created by Sarthak on 10/4/2019.
 */
@Disabled

@Autonomous(name = "ReturnToStart")
public class chewy_ReturnToStart extends chewy_AutonomousMethods {

    @Override
    public void runOpMode() {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        Init();
        //tell the odometry where to start based on where auto ended
        initOdometryHardware(36,84,0);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();


        //return to start position
        goToPostion(40 * robot.COUNTS_PER_INCH,9 * robot.COUNTS_PER_INCH,0.5,0,3 * robot.COUNTS_PER_INCH,false);

        //Stop the thread
        robot.globalPositionUpdate.stop();

    }

}



