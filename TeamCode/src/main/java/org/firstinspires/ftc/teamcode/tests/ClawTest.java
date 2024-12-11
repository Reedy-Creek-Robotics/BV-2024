package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import java.security.cert.LDAPCertStoreParameters;
import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;


@TeleOp
public class ClawTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        Servo clawServo1 = hardwareMap.get(Servo.class, "ClawServo1");
        Servo clawServo2 = hardwareMap.get(Servo.class, "ClawServo2");

        clawServo1.setPosition(0.05);
        clawServo2.setPosition(1);

        ElapsedTime timeSinceRightBumperPressed = new ElapsedTime();


        boolean claw_state = false;

        waitForStart();

        while (opModeIsActive()) {


            if(gamepad1.right_bumper) {
                if(timeSinceRightBumperPressed.milliseconds() > 500) {
                    timeSinceRightBumperPressed.reset();
                    claw_state = !claw_state;
                }

            }
            if(claw_state) {

                clawServo1.setPosition(0);
                clawServo2.setPosition(1);
            }
            else {

                clawServo1.setPosition(1);
                clawServo2.setPosition(0);
            }


        } // end while loop
    }
}
// Trust omg ok trust