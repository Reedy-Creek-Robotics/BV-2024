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
                stop();
            }


            // takes 2 inputs, the turning speed and waiting time to stop
            public void TankTurn(double TurnVal, int wait){
                frontLeftMotor.setPower(TurnVal);
                frontRightMotor.setPower(TurnVal);
                backLeftMotor.setPower(TurnVal);
                backRightMotor.setPower(TurnVal);
                sleep(wait);
                stop();
            }
        }

        Movement inner = new Movement();





        waitForStart();



        telemetry.addData("running","True");
        telemetry.update();

        /*
         calls a inner class which is movement and gets the functions gives inputs and runs,
         this is where our main auto code will go
         */
        inner.Forward(0.6, 3000);
        inner.TankTurn(0.5,500);





    }
}