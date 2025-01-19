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
public class WHOLETRUST extends LinearOpMode {

    private DcMotor LeftLinearSlide;
    private DcMotor RightLinearSlide;

    private int aButtonState = 0; // Tracks the current step
    private ElapsedTime aButtonTimer = new ElapsedTime();

    private final int POSITION_HIGH = 3800;
    private final int POSITION_SPECIMEN_HIGH = 2450;
    private final int POSITION_SPECIMEN_LOW = 1350;
    private final int POSITION_BASE = 0;
    private double state_1_1 = 0.49;
    private double state_1_2 = 0.51;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        LeftLinearSlide = hardwareMap.dcMotor.get("LeftLinear");
        RightLinearSlide = hardwareMap.dcMotor.get("RightLinear");

        // Reset encoders
        LeftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int count = 0;


        ElapsedTime timeSinceButtonPressed = new ElapsedTime();
        double LinearPower = 0.825;
        int linearState = 0;

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");


        Servo clawServo1 = hardwareMap.get(Servo.class, "ClawServo1");
        Servo clawServo2 = hardwareMap.get(Servo.class, "ClawServo2");
        Servo specServo1 = hardwareMap.get(Servo.class, "Spec1");
        Servo specServo2 = hardwareMap.get(Servo.class, "Spec2");

        Servo bucket = hardwareMap.get(Servo.class, "Bucket");

        Servo clawArm1 = hardwareMap.get(Servo.class, "ArmServo1");
        Servo clawArm2 = hardwareMap.get(Servo.class, "ArmServo2");

        clawServo1.setPosition(0.25);
        clawServo2.setPosition(0.75);

        ElapsedTime timeSinceRightBumperPressed = new ElapsedTime();
        ElapsedTime timeSinceLeftBumperPressed = new ElapsedTime();
        ElapsedTime timeSinceYButtonPressed = new ElapsedTime();
        ElapsedTime timeSinceAButtonPressed = new ElapsedTime();
        ElapsedTime timeSinceBButtonPressed = new ElapsedTime();

        boolean claw_state = false;
        boolean spec_state = true;
        boolean arm_state = true;
        boolean bucket_state = true;// false as open, true as closed//

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
            telemetry.addData("Left Motor Power", LeftLinearSlide.getPower());
            telemetry.addData("Right Motor Power", RightLinearSlide.getPower());
            telemetry.addData("Arm Position1", clawArm1.getPosition());
            telemetry.addData("Arm Position2", clawArm2.getPosition());
            telemetry.addData("Position", bucket.getPosition());
            telemetry.addData("Count:", count);
            telemetry.update();

            if ((gamepad1.dpad_up && timeSinceButtonPressed.milliseconds() > 1000) || (gamepad2.dpad_up && timeSinceButtonPressed.milliseconds() > 1000)) {
                linearState = 1; // Move up in states
                timeSinceButtonPressed.reset();
            }

            if ((gamepad1.dpad_down && timeSinceButtonPressed.milliseconds() > 1000) || (gamepad2.dpad_down && timeSinceButtonPressed.milliseconds() > 1000)) {
                linearState = 0;
                if ((LeftLinearSlide.getCurrentPosition() < 50) || (RightLinearSlide.getCurrentPosition() > -50)) {
                    LeftLinearSlide.setPower(0);
                    RightLinearSlide.setPower(0);

                }// Move down in states
                timeSinceButtonPressed.reset();
            }

            if ((gamepad1.dpad_right && timeSinceButtonPressed.milliseconds() > 1000) || (gamepad2.dpad_right && timeSinceButtonPressed.milliseconds() > 1000)) {
                linearState = 2;
                timeSinceButtonPressed.reset();
            }

            if ((gamepad1.dpad_left && timeSinceButtonPressed.milliseconds() > 1000) || (gamepad2.dpad_left && timeSinceButtonPressed.milliseconds() > 1000)) {
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

            if(gamepad1.right_bumper || gamepad2.right_bumper) {
                if(timeSinceRightBumperPressed.milliseconds() > 500) {
                    timeSinceRightBumperPressed.reset();
                    claw_state = !claw_state;
                }

            }

            if(gamepad1.y || gamepad2.y) {
                if (timeSinceYButtonPressed.milliseconds() > 500) {
                    timeSinceYButtonPressed.reset();
                    spec_state = !spec_state;
                }
            }

            if (gamepad1.a && timeSinceAButtonPressed.milliseconds() > 500) {
                timeSinceAButtonPressed.reset(); // Reset debounce timer
                if (aButtonState == 0) {
                    aButtonState = 1; // Start the sequence
                    aButtonTimer.reset(); // Reset the timer for the sequence
                }
            }

            // Sequential logic for aButtonState
            switch (aButtonState) {
                case 0:
                    // Waiting for gamepad1.a to be pressed
                    break;

                case 1:
                    // Step 1: Bring the arm back up near the bucket
                    arm_state = true;
                    state_1_1 = 0.49;
                    state_1_2 = 0.51;
                    clawArm1.setPosition(state_1_1);
                    clawArm2.setPosition(state_1_2);// May have to change to state_1_1 = 0.6 and state_1_2 = 0.4// Release the claw
                    if (aButtonTimer.seconds() > 1) { // Wait 1 second
                        aButtonState = 2; // Move to next step
                        aButtonTimer.reset(); // Reset timer
                    }

                    break;

                case 2:
                    sleep(100);
                    claw_state = false;
                    if (aButtonTimer.seconds() > 1) { // Wait 1 second
                        aButtonState = 3; // Move to next step
                        aButtonTimer.reset(); // Reset timer
                    }
                    break;

                case 3:
                    // Step 2: Put the arm back down
                    state_1_1 = 0.25; // Adjust servo positions
                    state_1_2 = 0.75;
                    clawArm1.setPosition(state_1_1);
                    clawArm2.setPosition(state_1_2);
                    if (aButtonTimer.seconds() > 1) { // Wait 1 second
                        aButtonState = 4; // Move to next step
                        aButtonTimer.reset(); // Reset timer
                    }
                    break;

                case 4:
                    // Step 3: Bring linear slides up
                    linearState = 1;
                    aButtonState = 0;
                    break;
            }

            if (gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5) {
                state_1_1 = Math.min(state_1_1 + 0.015, 1.0); // Ensure within bounds
                state_1_2 = Math.max(state_1_2 - 0.015, 0.0); // Ensure within bounds
                clawArm1.setPosition(state_1_1);
                clawArm2.setPosition(state_1_2);
            }

            if (gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5) {
                state_1_1 = Math.max(state_1_1 - 0.015, 0.0); // Ensure within bounds
                state_1_2 = Math.min(state_1_2 + 0.015, 1.0); // Ensure within bounds
                clawArm1.setPosition(state_1_1);
                clawArm2.setPosition(state_1_2);
            }

            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                if (timeSinceLeftBumperPressed.milliseconds() > 500) {
                    timeSinceLeftBumperPressed.reset();
                    arm_state = !arm_state;
                    if (arm_state) {
                        // Close position
                        state_1_1 = 0.49;
                        state_1_2 = 0.51;
                    } else {
                        // Open position
                        state_1_1 = 0;
                        state_1_2 = 1;
                    }
                }
            }

            if (gamepad1.b || gamepad2.b) {
                if (timeSinceBButtonPressed.milliseconds() > 500) {
                    timeSinceBButtonPressed.reset();
                    bucket_state = !bucket_state;
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

            frontLeftMotor.setPower(frontLeftPower*0.6);
            backLeftMotor.setPower(backLeftPower*0.6);
            frontRightMotor.setPower(frontRightPower*0.6);
            backRightMotor.setPower(backRightPower*0.6);



            if(claw_state) {

                clawServo1.setPosition(0.50);
                clawServo2.setPosition(0.50);
            }
            else {

                clawServo1.setPosition(0.25);
                clawServo2.setPosition(0.75);
            }

            if (spec_state) {

                specServo1.setPosition(0);
                specServo2.setPosition(1);

            }

            else {

                specServo1.setPosition(0.5);
                specServo2.setPosition(0.5);

            }


            if (bucket_state) {
                bucket.setPosition(0.25);
            }
            else {
                bucket.setPosition(0.46);
            }



        } // end while loop
    }
}
// Trust omg ok trust