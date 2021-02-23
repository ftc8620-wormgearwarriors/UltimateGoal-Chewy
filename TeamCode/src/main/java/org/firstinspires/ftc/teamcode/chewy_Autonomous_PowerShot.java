package org.firstinspires.ftc.teamcode;
import android.graphics.Bitmap;
import android.widget.EditText;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RingDetector;
import org.firstinspires.ftc.teamcode.RingDetectorParams;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;
import java.util.Locale;

@Autonomous(name = "chewy_Autonomous_PowerShot")

public class chewy_Autonomous_PowerShot extends chewy_AutonomousMethods{

    @Override
    public void runOpMode() {
        //Initialize hardware map values

        //initializing odometry hardware
        Init();
        initOdometryHardware(55, 9, 0);

        //letting cam init fully and telling driver not to start
        telemetry.addData(">", "DO NOT START YET");
        telemetry.update();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // wait for start of opmode
        waitForStart();

        // set robot speed
        double dRobotPower = 0.9;

        //drive to powershot location
        goToPostion(60 * robot.COUNTS_PER_INCH, 64 * robot.COUNTS_PER_INCH, dRobotPower, 0, 3 * robot.COUNTS_PER_INCH, false);
    }


}
