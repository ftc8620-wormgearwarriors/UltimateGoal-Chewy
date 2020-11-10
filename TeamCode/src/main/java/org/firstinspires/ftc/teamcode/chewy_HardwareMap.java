package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;



public class chewy_HardwareMap
{
    /* Public Sensors */
    public WGWIMU2018 imu;
    public BNO055IMU wgwIMU2018        = null;



    /* Public Motors */

    public DcMotor  frontLeftDrive         = null;
    public DcMotor  frontRightDrive        = null;
    public DcMotor  backLeftDrive          = null;
    public DcMotor  backRightDrive         = null;
    public DcMotor  shooterLeft            = null;
    public DcMotor  shooterRight           = null;
    public DcMotor  intake                 = null;


    /* Public Servos */

    public Servo intakeRoller              = null;
    public Servo firstTransfer             = null;
    public Servo secondTransfer            = null;
    public Servo  wobbleGrabberArm         = null;
    public Servo  wobbleGrabberClaw        = null;


    //public sensors

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public chewy_HardwareMap(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        wgwIMU2018 = hwMap.get(BNO055IMU.class, "imu");
        imu = new WGWIMU2018(wgwIMU2018);

        // Define and Initialize Motors

        frontLeftDrive  = hwMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hwMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hwMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hwMap.get(DcMotor.class, "backRightDrive");
        shooterLeft     = hwMap.get(DcMotor.class, "shooterLeft");
        shooterRight    = hwMap.get(DcMotor.class, "shooterRight");
        intake          = hwMap.get(DcMotor.class, "intake");


        // Define and Initialize Servos

        intakeRoller   = hwMap.get(Servo.class, "intakeRoller");
        firstTransfer   = hwMap.get(Servo.class, "firstTransfer");
        secondTransfer  = hwMap.get(Servo.class, "secondTransfer");
        wobbleGrabberArm = hwMap.get(Servo.class, "wobbleGrabberArm");
        wobbleGrabberClaw  = hwMap.get(Servo.class, "wobbleGrabberClaw");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);  // Set to FORWARD if using AndyMark motors
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);   // Set to FORWARD if using AndyMark motors
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);  // Set to FORWARD if using AndyMark motors

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        intake.setPower(0);
        intakeRoller.setPosition(0.5);
        firstTransfer.setPosition(0.5);
        secondTransfer.setPosition(0.5);
        wobbleGrabberArm.setPosition(0);
        wobbleGrabberClaw.setPosition(0);




        //resets motor encoders to zero


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }
    // Odometery Section
    public DcMotor verticalLeft     = null,
                   verticalRight    = null,
                   horizontal       = null;
    final double COUNTS_PER_INCH = 1714;

    // String verticalLeftEncoderName = "vle", verticalRightEncoderName = "vre", horizontalEncoderName = "he";
    //String rfName = "frontRightDrive", rbName = "backRightDrive", lfName = "frontLeftDrive", lbName = "backLeftDrive";
    //  String verticalLeftEncoderName = "frontRightDrive", verticalRightEncoderName = backRightDrive, horizontalEncoderName = "IntakeRight";
    OdometryGlobalCoordinatePosition globalPositionUpdate;
    Thread positionThread = null;

}


