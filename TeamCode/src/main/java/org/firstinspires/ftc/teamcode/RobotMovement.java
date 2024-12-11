package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotMovement {

    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;
    private final LinearOpMode opMode;
    private final IMU imu;
    Servo activeTake;

    private static final double DRIVE_POWER = 0.6;

    public RobotMovement(Servo activeInTake, DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight, IMU imu, LinearOpMode opMode) {
        this.frontLeftMotor = frontLeft;
        this.backLeftMotor = backLeft;
        this.frontRightMotor = frontRight;
        this.backRightMotor = backRight;
        this.opMode = opMode;
        this.imu = imu;
        this.activeTake = activeInTake;

        resetEncoders();
    }

    private void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setup() {
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void forward(int ticks) {
        moveWithDriftCorrection(ticks, ticks, ticks, ticks);
    }

    public void backward(int ticks) {
        moveWithDriftCorrection(-ticks, -ticks, -ticks, -ticks);
    }

    public void strafeLeft(int ticks) {
        moveWithDriftCorrection(-ticks, ticks, ticks, -ticks);
    }

    public void strafeRight(int ticks) {
        moveWithDriftCorrection(ticks, -ticks, -ticks, ticks);
    }

    public void turn(int degrees) {
        int turnTicks = degrees * 10; // Example conversion factor
        move(-turnTicks, -turnTicks, turnTicks, turnTicks);
    }

    private void move(int frontLeftInches, int backLeftInches, int frontRightInches, int backRightInches) {
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + inchesToTicks(frontLeftInches));
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + inchesToTicks(backLeftInches));
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + inchesToTicks(frontRightInches));
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + inchesToTicks(backRightInches));

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(DRIVE_POWER);
        backLeftMotor.setPower(DRIVE_POWER);
        frontRightMotor.setPower(DRIVE_POWER);
        backRightMotor.setPower(DRIVE_POWER);

        while (opMode.opModeIsActive() &&
                (frontLeftMotor.isBusy() || backLeftMotor.isBusy() ||
                        frontRightMotor.isBusy() || backRightMotor.isBusy())) {
            // Wait for motors to finish
        }

        stopMotors();
        opMode.sleep(100); // Short pause to stabilize before next movement
    }

    private void moveWithDriftCorrection(int frontLeftTicks, int backLeftTicks, int frontRightTicks, int backRightTicks) {
        double initialYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        move(frontLeftTicks, backLeftTicks, frontRightTicks, backRightTicks);

        double finalYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        correctDrift(initialYaw, finalYaw);
    }

    private void correctDrift(double initialYaw, double finalYaw) {
        double drift = finalYaw - initialYaw;

        if (Math.abs(drift) > 1) { // Correct if drift exceeds 1 degree
            double correctionSpeed = 0.3; // Speed for correction
            int correctionTime = (int) Math.abs(drift * 50); // Adjust timing based on drift

            if (drift > 0) {
                // Drifted right, turn left slightly
                turnTank(-correctionSpeed, correctionTime);
            } else {
                // Drifted left, turn right slightly
                turnTank(correctionSpeed, correctionTime);
            }
        }
    }

    private void turnTank(double power, int time) {
        frontLeftMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);

        opMode.sleep(time); // Pause for the correction time
        stopMotors();
    }

    private int inchesToTicks(double distanceInches) {
        final double TICKS_PER_INCH = 42.8;
        return ((int) (distanceInches*TICKS_PER_INCH));
    }

    private void stopMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
