package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous
public class SampleAutoCode extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor LeftLinearSlide;
    private DcMotor RightLinearSlide;
    private IMU imu;
    private LinearOpMode opMode;
    private final int POSITION_BASE = 0;
    private final int POSITION_HIGH = 3800;



    @Override
    public void runOpMode() {
        // Hardware initialization
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        LeftLinearSlide = hardwareMap.dcMotor.get("LeftLinear");
        RightLinearSlide = hardwareMap.dcMotor.get("RightLinear");

        Servo clawServo1 = hardwareMap.get(Servo.class, "ClawServo1");
        Servo clawServo2 = hardwareMap.get(Servo.class, "ClawServo2");
        Servo specServo1 = hardwareMap.get(Servo.class, "Spec1");
        Servo specServo2 = hardwareMap.get(Servo.class, "Spec2");

        Servo bucket = hardwareMap.get(Servo.class, "Bucket");

        Servo clawArm1 = hardwareMap.get(Servo.class, "ArmServo1");
        Servo clawArm2 = hardwareMap.get(Servo.class, "ArmServo2");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        int linearState = 0;


        // Reset encoders
        LeftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RobotMovement2 movement = new RobotMovement2(
                LeftLinearSlide,
                RightLinearSlide,
                frontLeftMotor,
                backLeftMotor,
                frontRightMotor,
                backRightMotor,
                imu,
                this);
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
            }
        LeftLinearSlide.setPower(0.7);
        RightLinearSlide.setPower(0.7);

        LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        movement.setup();

        movement.forward(12);
        movement.strafeLeft(2560);
        LeftLinearSlide.setTargetPosition(POSITION_HIGH);
        RightLinearSlide.setTargetPosition(POSITION_HIGH);
        bucket.setPosition(0.68);
        sleep(1000);
        bucket.setPosition(0.34);
        LeftLinearSlide.setTargetPosition(POSITION_BASE);
        RightLinearSlide.setTargetPosition(POSITION_BASE);
        movement.forward(24);
        sleep(1000);
        clawArm1.setPosition(1);
        clawArm2.setPosition(1);
        clawServo1.setPosition(0.75);
        clawServo2.setPosition(0.25);
        clawArm1.setPosition(0.25);
        clawArm2.setPosition(0.75);
        bucket.setPosition(0.34);
        clawServo1.setPosition(0.25);
        clawServo2.setPosition(0.75);
        movement.backward(3414);
        sleep(1000);
        LeftLinearSlide.setTargetPosition(POSITION_HIGH);
        RightLinearSlide.setTargetPosition(POSITION_HIGH);
        bucket.setPosition(0.68);
        bucket.setPosition(0.34);
        LeftLinearSlide.setTargetPosition(POSITION_BASE);
        RightLinearSlide.setTargetPosition(POSITION_BASE);
       // sleep(1000);
        movement.forward(24);
        movement.strafeRight(427);
        sleep(1000);
        clawArm1.setPosition(1);
        clawArm2.setPosition(1);
        clawServo1.setPosition(0.75);
        clawServo2.setPosition(0.25);
        clawArm1.setPosition(0.25);
        clawArm2.setPosition(0.75);
        bucket.setPosition(0.34);
        clawServo1.setPosition(0.25);
        clawServo2.setPosition(0.75);
        movement.backward(3414);
        movement.strafeLeft(427);
        sleep(1000);
        LeftLinearSlide.setTargetPosition(POSITION_HIGH);
        RightLinearSlide.setTargetPosition(POSITION_HIGH);
        bucket.setPosition(0.68);
        bucket.setPosition(0.34);
        LeftLinearSlide.setTargetPosition(POSITION_BASE);
        RightLinearSlide.setTargetPosition(POSITION_BASE);
        movement.forward(24);
        movement.strafeLeft(427);
        sleep(1000);
        clawArm1.setPosition(1);
        clawArm2.setPosition(1);
        clawServo1.setPosition(0.75);
        clawServo2.setPosition(0.25);
        clawArm1.setPosition(0.25);
        clawArm2.setPosition(0.75);
        bucket.setPosition(0.34);
        clawServo1.setPosition(0.25);
        clawServo2.setPosition(0.75);
        movement.backward(3414);
        movement.strafeRight(427);
        sleep(1000);
        LeftLinearSlide.setTargetPosition(POSITION_HIGH);
        RightLinearSlide.setTargetPosition(POSITION_HIGH);
        bucket.setPosition(0.68);
        bucket.setPosition(0.34);
        LeftLinearSlide.setTargetPosition(POSITION_BASE);
        RightLinearSlide.setTargetPosition(POSITION_BASE);



    }
    }