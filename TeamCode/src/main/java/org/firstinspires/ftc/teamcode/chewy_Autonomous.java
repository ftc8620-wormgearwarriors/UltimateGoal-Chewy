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
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values.
        Init();
        initOdometryHardware(0, 111, 90);


    }
}
