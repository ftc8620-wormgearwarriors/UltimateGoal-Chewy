package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

    @Autonomous(name = "chewy_Vuforia")
    public class chewy_Vuforia extends chewy_AutonomousMethods {


        @Override
        public void runOpMode() {
            //Initialize hardware map values.
            Init();
            initOdometryHardware(0, 111, 90);
            VuforiaInit();


        }
    }


