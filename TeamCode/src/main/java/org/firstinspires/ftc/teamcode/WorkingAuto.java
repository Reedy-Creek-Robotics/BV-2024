package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class WorkingAuto extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor LeftLinearSlide;
    private DcMotor RightLinearSlide;
    private IMU imu;
    private LinearOpMode opMode;
    private final int POSITION_HIGH = 3800;
    private final int POSITION_SPECIMEN_HIGH = 2100;
    private final int POSITION_SPECIMEN_LOW = 1330;
    private final int POSITION_BASE = 0;

    @Override
    public void runOpMode() {
        // Hardware initialization
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        LeftLinearSlide = hardwareMap.dcMotor.get("LeftLinear");


        Servo clawServo1 = hardwareMap.get(Servo.class, "ClawServo1");
        Servo clawServo2 = hardwareMap.get(Servo.class, "ClawServo2");
        Servo specServo1 = hardwareMap.get(Servo.class, "Spec1");
        Servo specServo2 = hardwareMap.get(Servo.class, "Spec2");

        Servo bucket = hardwareMap.get(Servo.class, "Bucket");

        Servo clawArm1 = hardwareMap.get(Servo.class, "ArmServo1");
        Servo clawArm2 = hardwareMap.get(Servo.class, "ArmServo2");

        int linearState = 0;

        RightLinearSlide = hardwareMap.dcMotor.get("RightLinear");

        // Reset encoders
        LeftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        RobotMovement2 movement = new RobotMovement2(LeftLinearSlide, RightLinearSlide, frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, imu, this);
        //RobotMovement movement = new RobotMovement(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, imu, this);

        movement.setup();

        waitForStart();

        telemetry.addData("FrontLeft", frontLeftMotor.getCurrentPosition());
        telemetry.addData("FrontRight", frontRightMotor.getCurrentPosition());
        telemetry.addData("BackLeft", backLeftMotor.getCurrentPosition());
        telemetry.addData("BackRight", backRightMotor.getCurrentPosition());
        telemetry.update();


        clawServo1.setPosition(0.25);
        clawServo2.setPosition(0.75);

        specServo1.setPosition(0.5);
        specServo2.setPosition(0.5);

        clawArm1.setPosition(0.25);
        clawArm2.setPosition(0.75);

        bucket.setPosition(0.34);

        clawArm1.setPosition(0.4);
        clawArm2.setPosition(0.5);

        bucket.setPosition(0.34);

        switch (linearState) {
            // Left is port 1
            // Right is port 0
            case 0:
                telemetry.addData("Linear Slide Height:", "Base Level (0)");
                LeftLinearSlide.setTargetPosition(POSITION_BASE);
                RightLinearSlide.setTargetPosition(-POSITION_BASE);
                break;
            case 1:
                telemetry.addData("Linear Slide Height:", "High Basket");
                LeftLinearSlide.setTargetPosition(POSITION_HIGH);
                RightLinearSlide.setTargetPosition(-POSITION_HIGH);
                break;
            case 2:
                telemetry.addData("Linear Slide Height:", "High Specimen Level");
                LeftLinearSlide.setTargetPosition(POSITION_SPECIMEN_HIGH);
                RightLinearSlide.setTargetPosition(-POSITION_SPECIMEN_HIGH);
                break;
            case 3:
                telemetry.addData("Linear Slide Height:", "Low Specimen Level");
                LeftLinearSlide.setTargetPosition(POSITION_SPECIMEN_LOW);
                RightLinearSlide.setTargetPosition(-POSITION_SPECIMEN_LOW);
                break;
        }

        LeftLinearSlide.setPower(0.7);
        RightLinearSlide.setPower(0.7);

        LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        movement.setup();// Anubhav and Abhinav did all the auto code chat
        LeftLinearSlide.setTargetPosition(POSITION_SPECIMEN_HIGH);
        RightLinearSlide.setTargetPosition(-POSITION_SPECIMEN_HIGH);
        movement.backward(28.3525);
        sleep(1250);
        // Supposed to be 4.69, Changed for testing purposes
        specServo1.setPosition(0);
        specServo2.setPosition(1);
        movement.forward(9);
        LeftLinearSlide.setTargetPosition(0);// Alex plays games
        RightLinearSlide.setTargetPosition(0);
        movement.strafeLeft(39);
        movement.backward(37);
        movement.strafeLeft(15.5);
        movement.turn(47);
        movement.backward(59.8);
        specServo1.setPosition(0.5);
        specServo2.setPosition(0.5);
        sleep(300); // Can be 200
        LeftLinearSlide.setTargetPosition(POSITION_SPECIMEN_HIGH);
        RightLinearSlide.setTargetPosition(-POSITION_SPECIMEN_HIGH);
        sleep(999);
         // Vivaan le Goat did something chat
        movement.strafeLeft(52.5);
        movement.turn(46);
        movement.backward(18.45);
        LeftLinearSlide.setTargetPosition(POSITION_SPECIMEN_LOW);
        RightLinearSlide.setTargetPosition(-POSITION_SPECIMEN_LOW);
        sleep(1000); // Can be 200
        specServo1.setPosition(0);
        specServo2.setPosition(1);
        movement.forward(16.4);
        LeftLinearSlide.setTargetPosition(0);
        RightLinearSlide.setTargetPosition(0);
        movement.turn(4);
        movement.strafeLeft(59.8); // Sid is doing nothing
        movement.turn(46);
        sleep(200);
        movement.backward(3);

        sleep(200);
        specServo1.setPosition(0.5);//Mekhi's 1v4 clutch
        specServo2.setPosition(0.5);
        sleep(300);
        LeftLinearSlide.setTargetPosition(POSITION_SPECIMEN_HIGH);
        RightLinearSlide.setTargetPosition(-POSITION_SPECIMEN_HIGH);
        movement.strafeLeft(60.5);
        movement.turn(48.5);
        movement.backward(18); // Abhinav's quad Edit
        LeftLinearSlide.setTargetPosition(POSITION_SPECIMEN_LOW);
        RightLinearSlide.setTargetPosition(-POSITION_SPECIMEN_LOW);
        sleep(1000); // Can be 200
        specServo1.setPosition(0);
        specServo2.setPosition(1);
        movement.forward(21);
        movement.strafeLeft(56);






    }
}