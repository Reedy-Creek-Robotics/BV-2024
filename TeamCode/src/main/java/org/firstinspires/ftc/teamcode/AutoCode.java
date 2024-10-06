package org.firstinspires.ftc.teamcode;

import android.util.LruCache;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutoCode extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo clawServo1 = hardwareMap.get(Servo.class, "ClawServo1");
        Servo clawServo2 = hardwareMap.get(Servo.class, "ClawServo2");


        // class with all movement functions
        class Movement {

            // stop function/method which stops all motors
            public void Stop(){
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
            }


            // moves the motors at a speed for a set time and stops the motors
            public void Forward(double power, int wait) {
                frontLeftMotor.setPower(power);
                frontRightMotor.setPower(power);
                backLeftMotor.setPower(power);
                backRightMotor.setPower(power);
                sleep(wait);
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);

            }


            // takes 2 inputs, the turning speed and waiting time to stop, positive number means left turn and negative is right turn
            public void TankTurn(double TurnVal, int wait){
                frontLeftMotor.setPower(-TurnVal);
                frontRightMotor.setPower(TurnVal);
                backLeftMotor.setPower(-TurnVal);
                backRightMotor.setPower(TurnVal);
                sleep(wait);
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
            }

            // takes 2 inputs, the movement speed and waiting time to stop, positive number means stafe left, negative number means stafe right
            public void Strafe(double Speed, int wait){
                frontLeftMotor.setPower(Speed);
                frontRightMotor.setPower(-Speed);
                backLeftMotor.setPower(-Speed);
                backRightMotor.setPower(Speed);
                sleep(wait);
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
            }


        }

        Movement inner = new Movement();

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);




        waitForStart();



        telemetry.addData("running","True");
        telemetry.addData("Gyro",imu.getRobotYawPitchRollAngles());
        telemetry.update();

        /*
         calls a inner class which is movement and gets the functions gives inputs and runs,
         this is where our main auto code will go
         */

        // should just go forward turn left, strafe left and turn again, need to perfect turn vals to make sure it turns good

        inner.Forward(0.3, 2000);
        // wait for motors to stop
        sleep(500);
        inner.TankTurn(0.3,1260);
        // wait for motors to stop
        sleep(500);
        inner.Strafe(0.3,2000);
        sleep(500);
        inner.TankTurn(-0.3,1260);
        telemetry.addData("Gyro",imu.getRobotYawPitchRollAngles());
        telemetry.update();





    }
}