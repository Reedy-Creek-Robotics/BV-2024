package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot{

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor armMotor;
    DcMotor leftLinearSlide;
    DcMotor rightLinearSlide;
    Servo activeIntake;
    IMU imu;
    LinearOpMode opMode;
    private static final double DRIVE_POWER = 0.7;
    public Robot(
            DcMotor frontLeftMotor,
            DcMotor backLeftMotor,
            DcMotor frontRightMotor,
            DcMotor backRightMotor,
            DcMotor armMotor,
            DcMotor leftLinearSlide,
            DcMotor rightLinearSlide,
            Servo activeIntake,
            LinearOpMode opMode,
            IMU imu
    ) {
//        this.frontLeftMotor = frontLeftMotor;
//        this.backLeftMotor = backLeftMotor;
//        this.frontRightMotor = frontRightMotor;
//        this.backRightMotor = backRightMotor;
        this.armMotor = armMotor;
        this.leftLinearSlide = leftLinearSlide;
        this.rightLinearSlide = rightLinearSlide;
        this.activeIntake = activeIntake;
        this.imu = imu;
        this.opMode = opMode;
    }
    private void setup() {

        // Behavior when motor stops
        frontLeftMotor.setDirection(REVERSE);
        backLeftMotor.setDirection(REVERSE);
        frontLeftMotor.setZeroPowerBehavior(BRAKE);
        frontRightMotor.setZeroPowerBehavior(BRAKE);
        backLeftMotor.setZeroPowerBehavior(BRAKE);
        backRightMotor.setZeroPowerBehavior(BRAKE);
        activeIntake.setPosition(0.5);
        leftLinearSlide.setZeroPowerBehavior(BRAKE);
        rightLinearSlide.setDirection(REVERSE);

    }

    private int inchesToTicks(double distanceInches) {
        final double TICKS_PER_INCH = 0.02854620247;
        return ((int) (distanceInches*TICKS_PER_INCH));
    }
    // General Move Function

    public void forward(double distanceInches, double speed) {
        move(speed, distanceInches, distanceInches, distanceInches, distanceInches);
    }

    public void backward( double distanceInches, double speed) {
        move(speed, -distanceInches, -distanceInches, -distanceInches, -distanceInches);
    }

    public void strafeLeft(double distanceInches, double speed){
        move(speed,distanceInches,-distanceInches,distanceInches, -distanceInches);
    }

    public void strafeRight(double distanceInches, double speed){
        move(speed, -distanceInches, distanceInches,-distanceInches, distanceInches);
    }

    public void move(double speed, double frontLeftTicksInInches, double frontRightTicksInInches, double backLeftTicksInInches, double backRightTicksInInches) {
        setup();

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + inchesToTicks(frontLeftTicksInInches));
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + inchesToTicks(frontRightTicksInInches));
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + inchesToTicks(backLeftTicksInInches));
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + inchesToTicks(backRightTicksInInches));

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(DRIVE_POWER);
        frontRightMotor.setPower(DRIVE_POWER);
        backLeftMotor.setPower(DRIVE_POWER);
        backRightMotor.setPower(DRIVE_POWER);

    }

}

