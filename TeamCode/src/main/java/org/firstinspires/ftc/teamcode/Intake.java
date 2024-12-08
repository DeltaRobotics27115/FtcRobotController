package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class WristAndIntake extends LinearOpMode {
    public CRServo intake;

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(CRServo.class, "Intake");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();
        // Run until the end
        while (opModeIsActive()) {

            // Intake control with debugging
            if (gamepad2.left_bumper) {
                intake.setPower(1); // Move intake forward
                telemetry.addData("Intake Status", "Moving Forward");
            } else if (gamepad2.right_bumper) {
                intake.setPower(-1); // Move intake backward
                telemetry.addData("Intake Status", "Moving Backward");
            } else {
                intake.setPower(0); // Stop the intake when no bumper is pressed
                telemetry.addData("Intake Status", "Stopped");
            }

            // Display telemetry data
            telemetry.update();

        }
    }
}
