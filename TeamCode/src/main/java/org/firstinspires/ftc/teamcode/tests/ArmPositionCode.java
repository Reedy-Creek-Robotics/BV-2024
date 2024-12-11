package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class ArmPositionCode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Servo clawArm1 = hardwareMap.get(Servo.class, "ArmServo1");
        Servo clawArm2 = hardwareMap.get(Servo.class, "ArmServo2");

        ElapsedTime timeSinceLeftBumperPressed = new ElapsedTime();
        boolean arm_state = false;

        clawArm1.setPosition(0);// false as open, true as closed//
        clawArm2.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                if (timeSinceLeftBumperPressed.milliseconds() > 500) {
                    timeSinceLeftBumperPressed.reset();
                    arm_state = !arm_state;
                }
            }



            if (arm_state) {
                clawArm1.setPosition(1);
                clawArm2.setPosition(-1);
            }

            else {
                clawArm1.setPosition(0);
                clawArm2.setPosition(0);
            }



        } // end while loop
    }
}
// Trust omg ok trust