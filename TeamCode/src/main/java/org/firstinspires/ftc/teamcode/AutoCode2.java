package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class AutoCode2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo clawServo1 = hardwareMap.get(Servo.class, "ClawServo1");
        Servo clawServo2 = hardwareMap.get(Servo.class, "ClawServo2");

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Initialize the IMU with the parameters
        imu.initialize(parameters);

        // Class with all movement functions
        class Movement {

            // Stop function/method which stops all motors
            public void Stop() {
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
            }

            // Moves the motors at a speed for a set time and stops the motors
            public void Forward(double power, int wait) {
                double initialYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                frontLeftMotor.setPower(power);
                frontRightMotor.setPower(power);
                backLeftMotor.setPower(power);
                backRightMotor.setPower(power);
                sleep(wait);
                double finalYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                correctDrift(initialYaw, finalYaw);
                Stop();

            }

            // Takes 2 inputs, the turning speed and waiting time to stop, positive number means left turn and negative is right turn
            public void TankTurn(double TurnVal, int wait) {
                frontLeftMotor.setPower(-TurnVal);
                frontRightMotor.setPower(TurnVal);
                backLeftMotor.setPower(-TurnVal);
                backRightMotor.setPower(TurnVal);
                sleep(wait);
                Stop();
            }

            // Takes 2 inputs, the degrees to turn and speed to turn at, uses the IMU to make a tank turn more accuratly,
            // by running the motor until the angle is met
            public void GyroTurn(double TurnVal, int speed) {
                double Turn = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+TurnVal;
                frontLeftMotor.setPower(-speed);
                frontRightMotor.setPower(speed);
                backLeftMotor.setPower(-speed);
                backRightMotor.setPower(speed);
                int turn = 1;
                while(turn == 1){
                    if (TurnVal > 0){
                        if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) >= Turn){
                            turn = 0;
                            Stop();
                        }

                    }
                    if (TurnVal < 0){
                        if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) <= Turn){
                            turn = 0;
                            Stop();
                        }

                    }
                }
                Stop();
            }


            // Takes 2 inputs, the movement speed and waiting time to stop, positive number means strafe left, negative number means strafe right
            public void Strafe(double Speed, int wait) {
                double initialYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

                frontLeftMotor.setPower(Speed);
                frontRightMotor.setPower(-Speed);
                backLeftMotor.setPower(-Speed);
                backRightMotor.setPower(Speed);
                sleep(wait);
                double finalYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                correctDrift(initialYaw, finalYaw);
                Stop();

            }

            // Method to correct drift based on IMU readings
            private void correctDrift(double initialYaw, double finalYaw) {
                double drift = finalYaw - initialYaw;

                // If there's a significant drift, correct it with a small turn
                /*if (Math.abs(drift) > 0) {
                    double correctionSpeed = 0.1; // Adjust this value as necessary
                    if (drift > 0.5) {
                        // Drifted right, so turn left slightly
                        TankTurn(-correctionSpeed, (int) Math.abs(drift * 10)); // Adjust timing
                    } else {
                        // Drifted left, so turn right slightly
                        TankTurn(correctionSpeed, (int) Math.abs(drift * 10)); // Adjust timing
                    }
                }*/
            }
        }

        Movement inner = new Movement();

        waitForStart();

        telemetry.addData("running", "True");
        telemetry.addData("Gyro", imu.getRobotYawPitchRollAngles());
        telemetry.update();

        // Autonomous routine with drift correction
        // S1
        inner.Strafe(-0.4, 3000);

        // S3


    }
}