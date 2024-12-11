package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class MecanumAutoDrive extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private IMU imu;
    private LinearOpMode opMode;

    @Override
    public void runOpMode() {
        // Hardware initialization
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        Servo activeTake = hardwareMap.get(Servo.class, "ActiveTake");

        imu = hardwareMap.get(IMU.class, "imu");

        RobotMovement movement = new RobotMovement(activeTake, frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, imu, this);

        movement.forward(1);
    }
}