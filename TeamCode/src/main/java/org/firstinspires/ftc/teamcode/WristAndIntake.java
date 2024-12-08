package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class WristAndIntake extends LinearOpMode {
    public Servo wrist;
    public CRServo intake;

    @Override
    public void runOpMode() {
        wrist = hardwareMap.get(Servo.class, "Wrist");
        intake = hardwareMap.get(CRServo.class, "Intake");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();
        double wristcurrentposition = 0.5;
        while (opModeIsActive()) {

            // Wrist movement
            if (gamepad2.left_trigger > 0.2) {
                while (gamepad2.left_trigger > 0.2) {
                    wristcurrentposition = wristcurrentposition + 0.05;
                    telemetry.addData("Wrist Status", "Moving Left");
                    sleep(250);
                }
            } else if (gamepad2.right_trigger > 0.2) {
                while (gamepad2.right_trigger > 0.2) {
                    wristcurrentposition = wristcurrentposition - 0.05;
                    telemetry.addData("Wrist Status", "Moving Right");
                    sleep(250);
                }
            }
            
            wrist.setPosition(wristcurrentposition);

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
