package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class AutoCode extends LinearOpMode {

    private DcMotor LeftLinearSlide;
    private final int POSITION_LOW = 1600;
    private final int POSITION_HIGH = 3230;
    private final int POSITION_BASE = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        LeftLinearSlide = hardwareMap.dcMotor.get("LeftLinear");
        LeftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime timeSinceButtonPressed = new ElapsedTime();
        double LinearPower = 0.9;
        int linearState = 0;

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Motor Position", LeftLinearSlide.getCurrentPosition());
            telemetry.update();

            if (gamepad1.dpad_up && timeSinceButtonPressed.milliseconds() > 1000) {
                linearState = Math.min(linearState + 1, 2); // Move up in states
                timeSinceButtonPressed.reset();
            }

            if (gamepad1.dpad_down && timeSinceButtonPressed.milliseconds() > 1000) {
                linearState = Math.max(linearState - 1, 0); // Move down in states
                timeSinceButtonPressed.reset();
            }

            switch (linearState) {
                case 0:
                    telemetry.addData("Linear Slide Height:", "Base Level (-1542)");
                    LeftLinearSlide.setTargetPosition(POSITION_BASE);
                    LeftLinearSlide.setPower(LinearPower);
                    break;
                case 1:
                    telemetry.addData("Linear Slide Height:", "Low Basket Level (1600)");
                    LeftLinearSlide.setTargetPosition(POSITION_LOW);
                    LeftLinearSlide.setPower(LinearPower);
                    break;
                case 2:
                    telemetry.addData("Linear Slide Height:", "High Basket Level (3230)");
                    LeftLinearSlide.setTargetPosition(POSITION_HIGH);
                    LeftLinearSlide.setPower(LinearPower);
                    break;
            }

            // When the motor reaches the target position, stop it
            if (!LeftLinearSlide.isBusy()) {
                LeftLinearSlide.setPower(0); // Stop the motor when done
            }
        }
    }
}