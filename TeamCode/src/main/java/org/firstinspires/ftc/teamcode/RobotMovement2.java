package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotMovement2 {

    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;
    private final DcMotor Leftlin;
    private final DcMotor Rightlin;
    private final LinearOpMode opMode;
    private static final int TOLERANCE = 100;
    private final double Error_Margin = 25;
    private final IMU imu;
    Servo activeTake;

    private static final double DRIVE_POWER = 0.6;

    public RobotMovement2(DcMotor Leftlin, DcMotor Rightlin, DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight, IMU imu, LinearOpMode opMode) {
        this.frontLeftMotor = frontLeft;
        this.backLeftMotor = backLeft;
        this.frontRightMotor = frontRight;
        this.backRightMotor = backRight;
        this.Leftlin = Leftlin;
        this.Rightlin = Rightlin;
        this.opMode = opMode;
        this.imu = imu;

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

    public void setup() {
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void forward(double inch) {
        move(inch, inch, inch, inch);
    }

    public void backward(double ticks) {
        move(-ticks, -ticks, -ticks, -ticks);
    }

    public void strafeLeft(double ticks) {
        move(-ticks, ticks, ticks, -ticks);
    }

    public void strafeRight(double ticks) {
        move(ticks, -ticks, -ticks, ticks);
    }

    public void turn(double degrees) {
        double turnTicks = degrees; // Example conversion factor
        move(-turnTicks, -turnTicks, turnTicks, turnTicks);
    }

    // Trust

    public void strafeAtAngle(double angleDegrees, double distanceInches) {
        // Convert angle to radians for calculations
        double angleRadians = Math.toRadians(angleDegrees);

        // Calculate motor power ratios based on angle
        double frontLeftPower = Math.sin(angleRadians + Math.PI / 4);
        double frontRightPower = Math.cos(angleRadians + Math.PI / 4);
        double backLeftPower = Math.cos(angleRadians + Math.PI / 4);
        double backRightPower = Math.sin(angleRadians + Math.PI / 4);

        // Normalize the powers to maintain the correct ratios
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        frontLeftPower /= maxPower;
        frontRightPower /= maxPower;
        backLeftPower /= maxPower;
        backRightPower /= maxPower;

        // Calculate target ticks
        int ticks = inchesToTicks(distanceInches);

        // Set target positions
        int targetFrontLeft = frontLeftMotor.getCurrentPosition() + (int) (ticks * frontLeftPower);
        int targetFrontRight = frontRightMotor.getCurrentPosition() + (int) (ticks * frontRightPower);
        int targetBackLeft = backLeftMotor.getCurrentPosition() + (int) (ticks * backLeftPower);
        int targetBackRight = backRightMotor.getCurrentPosition() + (int) (ticks * backRightPower);

        frontLeftMotor.setTargetPosition(targetFrontLeft);
        frontRightMotor.setTargetPosition(targetFrontRight);
        backLeftMotor.setTargetPosition(targetBackLeft);
        backRightMotor.setTargetPosition(targetBackRight);

        // Set motors to RUN_TO_POSITION mode
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor powers
        frontLeftMotor.setPower(DRIVE_POWER * frontLeftPower);
        frontRightMotor.setPower(DRIVE_POWER * frontRightPower);
        backLeftMotor.setPower(DRIVE_POWER * backLeftPower);
        backRightMotor.setPower(DRIVE_POWER * backRightPower);

        boolean stopMoving = false;

        while (opMode.opModeIsActive() && !stopMoving) {
            boolean isFRDone = Math.abs(frontRightMotor.getCurrentPosition()) >= ((Math.abs(frontRightMotor.getTargetPosition())-Error_Margin));
            boolean isFLDone = Math.abs(frontLeftMotor.getCurrentPosition()) >= (Math.abs(frontLeftMotor.getTargetPosition())-Error_Margin);
            boolean isBRDone = Math.abs(backRightMotor.getCurrentPosition()) >= (Math.abs(backRightMotor.getTargetPosition()) - Error_Margin);
            boolean isBLDone = Math.abs(backLeftMotor.getCurrentPosition()) >= (Math.abs(backLeftMotor.getTargetPosition()) - Error_Margin);

            if(isFRDone && isFLDone && isBRDone && isBLDone) {
                stopMoving = true;
            }
            addTelemetry();
            // Optional: Update telemetry to monitor progress
        }

        stopMotors();

    }

    private void move(double frontLeftInches, double backLeftInches, double frontRightInches, double backRightInches) {
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

        boolean stopMoving = false;

        while (opMode.opModeIsActive() && !stopMoving) {
            boolean isFRDone = Math.abs(frontRightMotor.getCurrentPosition()) >= ((Math.abs(frontRightMotor.getTargetPosition())-Error_Margin));
            boolean isFLDone = Math.abs(frontLeftMotor.getCurrentPosition()) >= (Math.abs(frontLeftMotor.getTargetPosition())-Error_Margin);
            boolean isBRDone = Math.abs(backRightMotor.getCurrentPosition()) >= (Math.abs(backRightMotor.getTargetPosition()) - Error_Margin);
            boolean isBLDone = Math.abs(backLeftMotor.getCurrentPosition()) >= (Math.abs(backLeftMotor.getTargetPosition()) - Error_Margin);

            if(isFRDone && isFLDone && isBRDone && isBLDone) {
                stopMoving = true;
            }
            addTelemetry();
            // Optional: Update telemetry to monitor progress
        }

        stopMotors();
        // Short pause to stabilize before next movement
    }

    public void DriftCorrection() {
        double initialYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double finalYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        correctDrift(initialYaw, finalYaw);
    }

    private void addTelemetry() {
        opMode.telemetry.addData("FrontLeft", frontLeftMotor.getCurrentPosition());
        opMode.telemetry.addData("FrontRight", frontRightMotor.getCurrentPosition());
        opMode.telemetry.addData("BackLeft", backLeftMotor.getCurrentPosition());
        opMode.telemetry.addData("BackRight", backRightMotor.getCurrentPosition());
        opMode.telemetry.update();
    }

    private void correctDrift(double initialYaw, double finalYaw) {
        double drift = finalYaw - initialYaw;

        while (drift > 1) {
            drift = finalYaw - initialYaw;
            turn(1);
        }
        while (drift < -1) {
            drift = finalYaw - initialYaw;
            turn(-1);
        }
    }

    public void turnTank(double power, int time) {
        frontLeftMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);

        opMode.sleep(time); // Pause for the correction time
        stopMotors();
    }

    private int inchesToTicks(double distanceInches) {
        final double TICKS_PER_INCH = 42.8;
        return ((int) (distanceInches * TICKS_PER_INCH));
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
