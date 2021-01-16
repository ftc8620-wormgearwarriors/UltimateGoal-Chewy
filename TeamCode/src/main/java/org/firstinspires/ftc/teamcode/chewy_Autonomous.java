package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name = "chewy_Autonomous")
public class chewy_Autonomous extends chewy_AutonomousMethods {


    @Override
    public void runOpMode() {
        //Initialize hardware map values.
        Init();
        initOdometryHardware(24,  9 , 0);
        //VuforiaInit();

        //Identify number of rings



        //Drive to launch line
        goToPostion(30,70,0.5,0,1,false);

        //Shoot powershot targets




        //Park on launch line
        goToPostion(30,84,0.5,0,1,false);





    }
}
