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

    // boolean wasDown, wasUp;
    boolean flag_right_trigger = true;
    boolean LinearSlideForwardTelemetry, LinearSlideReversedTelemetry = true;


    DcMotor RFMotor;
    DcMotor RBMotor;
    DcMotor LFMotor;
    DcMotor LBMotor;

    /*

    DcMotor leftLinearSlide;
    DcMotor rightLinearSlide;

    */


    private Servo clawServo1;
    private Servo clawServo2;

    @Override
    public void init() {

        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");

        clawServo1 = hardwareMap.get(Servo.class, "clawServo1");
        clawServo2 = hardwareMap.get(Servo.class, "clawServo2");


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


        double vertical = -gamepad1.left_stick_y;
        double horizontal = gamepad1.left_stick_x * 1.1;
        double pivot = gamepad1.right_stick_x;

        float clawServoPosition = gamepad1.right_trigger;

        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

        RFMotor.setPower((pivot + (-vertical + horizontal)) / denominator);
        RBMotor.setPower((pivot + (-vertical - horizontal)) / denominator);
        LFMotor.setPower((pivot + (-vertical - horizontal)) / denominator);
        LBMotor.setPower((pivot + (-vertical + horizontal)) / denominator);

        // Assuming that both Claw Servo's are Standard Servo's

        if (gamepad1.right_trigger >= 0.5) {

            telemetry.addData("Claw", "Open");
            clawServo1.setPosition(0.6);
            // clawServo2.setPosition(0.6);

        }

        else {

            telemetry.addData("Claw", "Close");
            clawServo1.setPosition(0);
            // clawServo2.setPosition(1);
            // The second servo has to move in a opposite direction to the other servo

        }



        /*

        double linearSlidePower = 0.3;


        if (gamepad1.dpad_up) {
            telemetry.addData("Left and Right Linear Slide", "Activated (Forward)");
            leftLinearSlide.setPower(linearSlidePower);
            rightLinearSlide.setPower(linearSlidePower);
        }

        else if (gamepad1.dpad_down) {
            telemetry.addData("Left and Right Linear Slide", "Activated (Reverse)");
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

