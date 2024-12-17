package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.output.WristAndIntakePower;

public class WristAndIntakeControl {
    private Servo wrist;
    private CRServo intake;
    private double wristPosition = 0.5;
    private boolean intakeRunning = false; // Use a single flag for intake state
    private double intakePower = 0;

    public WristAndIntakeControl(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "Wrist");
        intake = hardwareMap.get(CRServo.class, "Intake");
    }

    public WristAndIntakePower update(double leftTrigger, double rightTrigger, boolean leftBumper, boolean rightBumper) {
        // Wrist movement (incremental with trigger hold)
        wristPosition += (leftTrigger > 0.2 ? 0.05 : 0) - (rightTrigger > 0.2 ? 0.05 : 0);
        wristPosition = Math.max(0.0, Math.min(1.0, wristPosition)); // Clamp wristPosition
        wrist.setPosition(wristPosition);

        // Intake control (simplified)
        if (leftBumper) {
            intakeRunning = true;
            intakePower = 1;
        } else if (rightBumper) {
            intakeRunning = true;
            intakePower = -1;
        } else if (intakeRunning) {
            intakeRunning = false;
            intakePower = 0;
        }

        intake.setPower(intakePower);
        return new WristAndIntakePower(wristPosition, intakePower);
    }


}
