package org.firstinspires.ftc.teamcode;

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
public class EntireRobot extends LinearOpMode {

    private DcMotor LeftLinearSlide;
    private DcMotor RightLinearSlide;

    private final int POSITION_HIGH = 3000;
    private final int POSITION_SPECIMEN_HIGH = 2450;
    private final int POSITION_SPECIMEN_LOW = 1350;
    private final int POSITION_BASE = 0;
    private double state_1_1 = 0.5;
    private double state_1_2 = 0.5;

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

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");


        Servo clawServo1 = hardwareMap.get(Servo.class, "ClawServo1");
        Servo clawServo2 = hardwareMap.get(Servo.class, "ClawServo2");
        Servo specServo1 = hardwareMap.get(Servo.class, "Spec1");
        Servo specServo2 = hardwareMap.get(Servo.class, "Spec2");

        Servo clawArm1 = hardwareMap.get(Servo.class, "ArmServo1");
        Servo clawArm2 = hardwareMap.get(Servo.class, "ArmServo2");

        Servo bucket = hardwareMap.get(Servo.class, "Bucket");

        clawServo1.setPosition(0.25);
        clawServo2.setPosition(0.75);

        ElapsedTime timeSinceRightBumperPressed = new ElapsedTime();
        ElapsedTime timeSinceLeftBumperPressed = new ElapsedTime();
        ElapsedTime timeSinceYButtonPressed = new ElapsedTime();
        ElapsedTime timeSinceAButtonPressed = new ElapsedTime();
        ElapsedTime timeSinceBButtonPressed = new ElapsedTime();

        boolean claw_state = false;
        boolean spec_state = true;
        boolean arm_state = true;// false as open, true as closed//

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            telemetry.addData("Left Motor Position", LeftLinearSlide.getCurrentPosition());
            telemetry.addData("Right Motor Position", RightLinearSlide.getCurrentPosition());
            telemetry.addData("Yaw angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("Position", bucket.getPosition());
            telemetry.update();

            if (gamepad1.dpad_up && timeSinceButtonPressed.milliseconds() > 1000) {
                linearState = 1; // Move up in states
                timeSinceButtonPressed.reset();
            }

            if (gamepad1.dpad_down && timeSinceButtonPressed.milliseconds() > 1000) {
                linearState = 0; // Move down in states
                timeSinceButtonPressed.reset();
            }

            if (gamepad1.dpad_right && timeSinceButtonPressed.milliseconds() > 1000) {
                linearState = 2;
                timeSinceButtonPressed.reset();
            }

            if (gamepad1.dpad_left && timeSinceButtonPressed.milliseconds() > 1000) {
                linearState = 3;
                timeSinceButtonPressed.reset();
            }

            // Set target positions based on the current state
            switch (linearState) {
                // Left is port 1
                // Right is port 0
                case 0:
                    telemetry.addData("Linear Slide Height:", "Base Level (0)");
                    LeftLinearSlide.setTargetPosition(POSITION_BASE);
                    RightLinearSlide.setTargetPosition(-POSITION_BASE);
                    break;
                case 1:
                    telemetry.addData("Linear Slide Height:", "High Basket");
                    LeftLinearSlide.setTargetPosition(POSITION_HIGH);
                    RightLinearSlide.setTargetPosition(-POSITION_HIGH);
                    break;
                case 2:
                    telemetry.addData("Linear Slide Height:", "High Specimen Level");
                    LeftLinearSlide.setTargetPosition(POSITION_SPECIMEN_HIGH);
                    RightLinearSlide.setTargetPosition(-POSITION_SPECIMEN_HIGH);
                    break;
                case 3:
                    telemetry.addData("Linear Slide Height:", "Low Specimen Level");
                    LeftLinearSlide.setTargetPosition(POSITION_SPECIMEN_LOW);
                    RightLinearSlide.setTargetPosition(-POSITION_SPECIMEN_LOW);
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

            if(gamepad1.right_bumper) {
                if(timeSinceRightBumperPressed.milliseconds() > 500) {
                    timeSinceRightBumperPressed.reset();
                    claw_state = !claw_state;
                }

            }

            if(gamepad1.y) {
                if (timeSinceYButtonPressed.milliseconds() > 500) {
                    timeSinceYButtonPressed.reset();
                    spec_state = !spec_state;
                }
            }

            if (gamepad1.left_trigger > 0.5) {
                state_1_1 = Math.min(state_1_1 + 0.01, 1.0); // Ensure within bounds
                state_1_2 = Math.max(state_1_2 - 0.01, 0.0); // Ensure within bounds
                clawArm1.setPosition(state_1_1);
                clawArm2.setPosition(state_1_2);
            }

            if (gamepad1.right_trigger > 0.5) {
                state_1_1 = Math.max(state_1_1 - 0.01, 0.0); // Ensure within bounds
                state_1_2 = Math.min(state_1_2 + 0.01, 1.0); // Ensure within bounds
                clawArm1.setPosition(state_1_1);
                clawArm2.setPosition(state_1_2);
            }

            if (gamepad1.left_bumper) {
                if (timeSinceLeftBumperPressed.milliseconds() > 500) {
                    timeSinceLeftBumperPressed.reset();
                    arm_state = !arm_state;
                    if (arm_state) {
                        // Close position
                        state_1_1 = 0.5;
                        state_1_2 = 0.5;
                    } else {
                        // Open position
                        state_1_1 = 0.04;
                        state_1_2 = 0.96;
                    }
                }
            }

            clawArm1.setPosition(state_1_1);
            clawArm2.setPosition(state_1_2);


            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);



            if(claw_state) {

                clawServo1.setPosition(0.6);
                clawServo2.setPosition(0.4);
            }
            else {

                clawServo1.setPosition(0.25);
                clawServo2.setPosition(0.75);
            }

            if (spec_state) {

                specServo1.setPosition(1);
                specServo2.setPosition(0);

            }

            else {

                specServo1.setPosition(0.5);
                specServo2.setPosition(0.5);

            }



        } // end while loop
    }
}
// Trust omg ok trust