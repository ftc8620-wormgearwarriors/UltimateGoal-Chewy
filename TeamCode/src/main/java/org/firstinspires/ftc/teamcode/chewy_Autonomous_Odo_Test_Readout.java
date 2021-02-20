package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "chewy_Autonomous_Odo_Test_Readout")
public class chewy_Autonomous_Odo_Test_Readout extends chewy_AutonomousMethods {


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

        while(opModeIsActive()) {
//            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
//            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
//            telemetry.addData("IMU Angle", robot.imu.getHeading());
            telemetry.addData("Vertical Left Position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", robot.verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", robot.horizontal.getCurrentPosition());

            telemetry.addData("X Position", robot.globalPositionUpdate.returnXCoordinate());
            telemetry.addData("Y Position", robot.globalPositionUpdate.returnYCoordinate());
            telemetry.addData("Heading", robot.globalPositionUpdate.returnOrientation());

            //           telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }

        //Stop the thread
        robot.globalPositionUpdate.stop();
    }



}
