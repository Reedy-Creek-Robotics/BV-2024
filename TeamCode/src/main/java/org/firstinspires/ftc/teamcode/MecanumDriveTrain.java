package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


// Akshay's edit


@TeleOp()

// “MecanumDriveTrain” should be the name of the file!

public class MecanumDriveTrain extends OpMode {

    // RF stands for Right Front
    // LB stands for Left Back

    boolean wasDown, wasUp;

    DcMotor RFMotor;
    DcMotor RBMotor;
    DcMotor LFMotor;
    DcMotor LBMotor;

    /*

    DcMotor leftLinearSlide;
    DcMotor rightLinearSlide;

    */


    @Override
    public void init() {

        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");

        Servo clawServo1 = hardwareMap.get(Servo.class, "clawServo1");
        Servo clawServo2 = hardwareMap.get(Servo.class, "clawServo2");

        /*

        leftLinearSlide = hardwareMap.get(DcMotor.class, "leftLinearSlide");
        rightLinearSlide = hardwareMap.get(DcMotor.class, "rightLinearSlide");

         */

        // This may have to be changed or deleted based on the Mecanum wheel orientation

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        double vertical = 0;
        double horizontal = 0;
        double pivot = 0;

        vertical = -gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;

        RFMotor.setPower(pivot + (-vertical + horizontal));
        RBMotor.setPower(pivot + (-vertical - horizontal));
        LFMotor.setPower(pivot + (-vertical - horizontal));
        LBMotor.setPower(pivot + (-vertical + horizontal));

        float clawServoPower = gamepad1.right_trigger;



        /*

        double linearSlidePower = 0.3;
        double clawServoPower = 0.3;

        float rightTriggerPressed = gamepad1.right_trigger;
        float leftTriggerPressed = gamepad1.left_trigger;

        if (rightTriggerPressed > 0.5) {
            telemetry.addData("Right Trigger", "Pressed");
        }

        if (leftTriggerPressed > 0.5) {
            telemetry.addData("Left Trigger", "Pressed");
        }


        if (gamepad1.dpad_up) {
            telemetry.addData("Left and Right Linear Slide", "Activated Forward");
            leftLinearSlide.setPower(linearSlidePower);
            rightLinearSlide.setPower(linearSlidePower);
        }

        else if (gamepad1.dpad_down) {
            telemetry.addData("Left and Right Linear Slide", "Activated Backward");
            leftLinearSlide.setPower(-linearSlidePower);
            rightLinearSlide.setPower(-linearSlidePower);
        }
        else {
            leftLinearSlide.setPower(0);
            rightLinearSlide.setPower(0);
        }

         */
    }

}

