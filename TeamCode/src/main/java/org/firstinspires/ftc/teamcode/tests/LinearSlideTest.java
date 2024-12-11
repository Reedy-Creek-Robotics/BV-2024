package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class LinearSlideTest extends LinearOpMode {

    private DcMotor LeftLinearSlide;
    private DcMotor RightLinearSlide;

    private final int POSITION_HIGH = 3050;
    private final int POSITION_BASE = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        LeftLinearSlide = hardwareMap.dcMotor.get("LeftLinear");
        RightLinearSlide = hardwareMap.dcMotor.get("RightLinear");

        // Reset encoders
        LeftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ElapsedTime timeSinceButtonPressed = new ElapsedTime();
        double LinearPower = 0.6;
        int linearState = 0;

        waitForStart();

        while (opModeIsActive()) {



            telemetry.addData("Left Motor Position", LeftLinearSlide.getCurrentPosition());
            telemetry.addData("Right Motor Position", RightLinearSlide.getCurrentPosition());
            telemetry.update();





            if (gamepad1.dpad_up && timeSinceButtonPressed.milliseconds() > 1000) {
                linearState = Math.min(linearState + 1, 2); // Move up in states
                timeSinceButtonPressed.reset();
            }

            if (gamepad1.dpad_down && timeSinceButtonPressed.milliseconds() > 1000) {
                linearState = Math.max(linearState - 1, 0); // Move down in states
                timeSinceButtonPressed.reset();
            }

            // Set target positions based on the current state
            switch (linearState) {
                // Left is port 1
                // Right is port 0
                case 0:
                    telemetry.addData("Linear Slide Height:", "Base Level (0)");
                    LeftLinearSlide.setTargetPosition(-POSITION_BASE);
                    RightLinearSlide.setTargetPosition(POSITION_BASE);
                    break;
                case 1:
                    telemetry.addData("Linear Slide Height:", "Low Basket Level (2400)");
                    LeftLinearSlide.setTargetPosition(-POSITION_HIGH);
                    RightLinearSlide.setTargetPosition(POSITION_HIGH);
                    break;
            }

            // Set motor mode to RUN_TO_POSITION
            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set power to move to the target position
            LeftLinearSlide.setPower(LinearPower);
            RightLinearSlide.setPower(LinearPower);

            // When both motors reach their target positions, stop them
            if (!LeftLinearSlide.isBusy() && !RightLinearSlide.isBusy()) {
                LeftLinearSlide.setPower(0);
                RightLinearSlide.setPower(0); // Stop the motors when done
            }


        } // end while loop
    }
}
// Trust omg ok trust