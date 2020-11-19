package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * Created by Sarthak on 10/4/2019.
 */
@Autonomous(name = "chewyAutoForce")
public class chewyAutoMeet1 extends chewy_AutonomousMethods {


    @Override
    public void runOpMode() {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        Init();
        initOdometryHardware(24,9,0);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();
//        while(opModeIsActive()){
//            //Display Global (x, y, theta) coordinates
//            telemetry.addData("X Position", robot.globalPositionUpdate.returnXCoordinate() / robot.COUNTS_PER_INCH);
//            telemetry.addData("Y Position", robot.globalPositionUpdate.returnYCoordinate() / robot.COUNTS_PER_INCH);
//            telemetry.addData("Orientation (Degrees)", robot.globalPositionUpdate.returnOrientation());
//            telemetry.addData("Thread Active", robot.positionThread.isAlive());
//            telemetry.addData("Vertical Left Position",  robot.verticalLeft.getCurrentPosition());
//            telemetry.addData("Vertical Right Position", robot.verticalRight.getCurrentPosition());
//            telemetry.addData("Horizontal Position",     robot.horizontal.getCurrentPosition());
//            telemetry.update();
//        }

        //drive to launch line
        goToPostion(17 * robot.COUNTS_PER_INCH,56 * robot.COUNTS_PER_INCH,0.5,0,3 * robot.COUNTS_PER_INCH,false);
        goToPostion(44 * robot.COUNTS_PER_INCH,72 * robot.COUNTS_PER_INCH,0.5,0,3 * robot.COUNTS_PER_INCH,false);
        goToPostion(44 * robot.COUNTS_PER_INCH,69 * robot.COUNTS_PER_INCH,0.5,0,3 * robot.COUNTS_PER_INCH,false);


        //shoot powershot targets
        firstDiskAuto();
        secondDiskAuto();
        thirdDiskAuto();

        //park on launch line
        goToPostion(36 * robot.COUNTS_PER_INCH,84 * robot.COUNTS_PER_INCH,0.5,0,3 * robot.COUNTS_PER_INCH,false);

        //Stop the thread
        robot.globalPositionUpdate.stop();

    }

}



