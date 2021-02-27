package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


/**
 * Created by Sarthak on 10/4/2019.
 */
@Disabled

@Autonomous(name = "Shooter Test")
public class chewy_ShooterTest extends chewy_AutonomousMethods {

    @Override
    public void runOpMode() {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        Init();
        initOdometryHardware(40,9,0);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //drive to launch line
//        goToPostion(17 * robot.COUNTS_PER_INCH,56 * robot.COUNTS_PER_INCH,0.5,0,3 * robot.COUNTS_PER_INCH,false);
//        goToPostion(44 * robot.COUNTS_PER_INCH,72 * robot.COUNTS_PER_INCH,0.5,0,3 * robot.COUNTS_PER_INCH,false);

        //shoot powershot targets
        //firstDiskAuto();
        //secondDiskAuto();
        //thirdDiskAuto();
        rapidFireDisks();

        //park on launch line
//        goToPostion(36 * robot.COUNTS_PER_INCH,84 * robot.COUNTS_PER_INCH,0.5,0,3 * robot.COUNTS_PER_INCH,false);

        //Stop the thread
        robot.globalPositionUpdate.stop();

    }

}



