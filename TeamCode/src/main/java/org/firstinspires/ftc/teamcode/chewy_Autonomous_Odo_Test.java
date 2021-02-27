package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
@Disabled

@Autonomous(name = "chewy_Autonomous_Odo_Test")
public class chewy_Autonomous_Odo_Test extends chewy_AutonomousMethods {


    @Override
    public void runOpMode() {
        //Initialize hardware map values

        //initializing odometry hardware
        Init();
        initOdometryHardware(55, 9, 0);




        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // wait for start of opmode
        waitForStart();


        //Display raw values
        telemetry.addData("Vertical Left Position", robot.verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right Position", robot.verticalRight.getCurrentPosition());
        telemetry.addData("Horizontal Position", robot.horizontal.getCurrentPosition());

        telemetry.addData("X Position", robot.globalPositionUpdate.returnXCoordinate());
        telemetry.addData("Y Position", robot.globalPositionUpdate.returnYCoordinate());
        telemetry.addData("Heading", robot.globalPositionUpdate.returnOrientation());


        //Update values
        telemetry.update();

        // set robot speed
        double dRobotPower = 0.9;

        //drive in diamond to test heading
        goToPostion(31 * robot.COUNTS_PER_INCH, 33 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);
        //Display raw values
        telemetry.addData("Vertical Left Position", robot.verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right Position", robot.verticalRight.getCurrentPosition());
        telemetry.addData("Horizontal Position", robot.horizontal.getCurrentPosition());

        telemetry.addData("X Position", robot.globalPositionUpdate.returnXCoordinate());
        telemetry.addData("Y Position", robot.globalPositionUpdate.returnYCoordinate());
        telemetry.addData("Heading", robot.globalPositionUpdate.returnOrientation());

        //Update values
        telemetry.update();

        sleep(3000);


        //2nd pos
        goToPostion(55 * robot.COUNTS_PER_INCH, 57 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);
        //Display raw values
        telemetry.addData("Vertical Left Position", robot.verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right Position", robot.verticalRight.getCurrentPosition());
        telemetry.addData("Horizontal Position", robot.horizontal.getCurrentPosition());

        telemetry.addData("X Position", robot.globalPositionUpdate.returnXCoordinate());
        telemetry.addData("Y Position", robot.globalPositionUpdate.returnYCoordinate());
        telemetry.addData("Heading", robot.globalPositionUpdate.returnOrientation());

        //Update values
        telemetry.update();

        sleep(3000);



        //3rd pos
        goToPostion(79 * robot.COUNTS_PER_INCH, 33 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);
        //Display raw values
        telemetry.addData("Vertical Left Position", robot.verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right Position", robot.verticalRight.getCurrentPosition());
        telemetry.addData("Horizontal Position", robot.horizontal.getCurrentPosition());

        telemetry.addData("X Position", robot.globalPositionUpdate.returnXCoordinate());
        telemetry.addData("Y Position", robot.globalPositionUpdate.returnYCoordinate());
        telemetry.addData("Heading", robot.globalPositionUpdate.returnOrientation());

        //Update values
        telemetry.update();

        sleep(3000);


        //4th pos
        goToPostion(55 * robot.COUNTS_PER_INCH, 9 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);
        //Display raw values
        telemetry.addData("Vertical Left Position", robot.verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right Position", robot.verticalRight.getCurrentPosition());
        telemetry.addData("Horizontal Position", robot.horizontal.getCurrentPosition());

        telemetry.addData("X Position", robot.globalPositionUpdate.returnXCoordinate());
        telemetry.addData("Y Position", robot.globalPositionUpdate.returnYCoordinate());
        telemetry.addData("Heading", robot.globalPositionUpdate.returnOrientation());

        //Update values
        telemetry.update();

        sleep(3000);



        // test pretending drop 1st wobble
        goToPostion(48 * robot.COUNTS_PER_INCH, 115 * robot.COUNTS_PER_INCH, dRobotPower,  0, 3 * robot.COUNTS_PER_INCH, false);

        //Drive to lanch line
        goToPostion(39 * robot.COUNTS_PER_INCH,65 * robot.COUNTS_PER_INCH, dRobotPower,0,3 * robot.COUNTS_PER_INCH, false);

        telemetry.addData("Vertical Left Position", robot.verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right Position", robot.verticalRight.getCurrentPosition());
        telemetry.addData("Horizontal Position", robot.horizontal.getCurrentPosition());

        telemetry.addData("X Position", robot.globalPositionUpdate.returnXCoordinate());
        telemetry.addData("Y Position", robot.globalPositionUpdate.returnYCoordinate());
        telemetry.addData("Heading", robot.globalPositionUpdate.returnOrientation());

        //           telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

        //Update values
        telemetry.update();
         sleep(8000);

            //shoot powershot targets
            //rapidFireDisks();

            //goes to pickup position and grabs goal
            goToPostion(21.5 * robot.COUNTS_PER_INCH, 37 * robot.COUNTS_PER_INCH, dRobotPower, 0, 1 * robot.COUNTS_PER_INCH, false);





        //Stop the thread
        robot.globalPositionUpdate.stop();
    }



}
