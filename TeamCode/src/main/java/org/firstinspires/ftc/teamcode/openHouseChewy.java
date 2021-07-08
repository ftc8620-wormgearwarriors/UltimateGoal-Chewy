package org.firstinspires.ftc.teamcode;
import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;
import java.util.Locale;


@Autonomous(name = "openHouseChewy")
public class openHouseChewy extends chewy_AutonomousMethods {
    @Override
    public void runOpMode() {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        Init();
        initOdometryHardware(36,9,0);
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //go forward to first turn
        goToPostion(36 * robot.COUNTS_PER_INCH,33 * robot.COUNTS_PER_INCH,0.5,0,3 * robot.COUNTS_PER_INCH,false);
      // goToPostion();






















        //turn and go to next turn
//        goToPostion(60 * robot.COUNTS_PER_INCH,33 * robot.COUNTS_PER_INCH,0.5,0,3 * robot.COUNTS_PER_INCH,false);
//        //turn and go to next
//        goToPostion(60 * robot.COUNTS_PER_INCH,81 * robot.COUNTS_PER_INCH,0.5,0,3 * robot.COUNTS_PER_INCH,false);
//        //turn and go to next turn
//        goToPostion(36 * robot.COUNTS_PER_INCH,81 * robot.COUNTS_PER_INCH,0.5,0,3 * robot.COUNTS_PER_INCH,false);
//        //forward to end
//        goToPostion(36 * robot.COUNTS_PER_INCH,129 * robot.COUNTS_PER_INCH,0.5,0,3 * robot.COUNTS_PER_INCH,false);

        //Stop the thread
        robot.globalPositionUpdate.stop();

    }


            }

