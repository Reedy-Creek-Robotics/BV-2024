package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class LinearSlides extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        DcMotor LeftLinearSlide = hardwareMap.dcMotor.get("LeftLinear");
        DcMotor RightLinearSlide = hardwareMap.dcMotor.get("RightLinear");

        double LinearSlidePower = 0.6;


        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                LeftLinearSlide.setPower(LinearSlidePower);
                RightLinearSlide.setPower(LinearSlidePower);
            }

            else if (gamepad1.dpad_down) {
                LeftLinearSlide.setPower(-LinearSlidePower);
                RightLinearSlide.setPower(-LinearSlidePower);
            }

            else {
                LeftLinearSlide.setPower(0);
                RightLinearSlide.setPower(0);
            }

        }

    }

}