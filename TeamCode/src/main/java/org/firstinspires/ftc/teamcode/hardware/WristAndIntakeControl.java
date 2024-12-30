package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.output.WristAndIntakePower;

/**
 * This class controls the wrist and intake mechanisms of the robot.
 */
public class WristAndIntakeControl {

    private static final double WRIST_INCREMENT = 0.01; // Define increment as a constant
    private static final double TRIGGER_THRESHOLD = 0.2; // Define trigger threshold as a constant

    private final Servo wrist;
    private final CRServo intake;

    private double wristPosition = 0.5;
    private double intakePower = 0; // Store intake power directly

    /**
     * Constructor for WristAndIntakeControl.
     * Initializes the wrist and intake hardware.
     *
     * @param hardwareMap The hardware map to access the wrist and intake.
     */
    public WristAndIntakeControl(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "Wrist");
        intake = hardwareMap.get(CRServo.class, "Intake");
    }

    /**
     * Updates the wrist and intake positions based on gamepad input.
     *
     * @param leftTrigger  The value of the left trigger.
     * @param rightTrigger The value of the right trigger.
     * @param leftBumper  The state of the left bumper.
     * @param rightBumper The state of the right bumper.
     * @return A WristAndIntakePower object containing the current wrist position and intake power.
     */
    public WristAndIntakePower update(double leftTrigger, double rightTrigger, boolean leftBumper, boolean rightBumper) {
        // Update wrist position based on triggers
        if (leftTrigger > TRIGGER_THRESHOLD) {
            wristPosition += WRIST_INCREMENT;
        } else if (rightTrigger > TRIGGER_THRESHOLD) {
            wristPosition -= WRIST_INCREMENT;
        }
        // Clamp wrist position to valid range
        wristPosition = Math.max(0.0, Math.min(1.0, wristPosition));
        wrist.setPosition(wristPosition);

        // Update intake power based on bumpers
        if (leftBumper) {
            intakePower = 1;
        } else if (rightBumper) {
            intakePower = -1;
        } else {
            intakePower = 0; // Stop intake if no bumpers are pressed
        }
        intake.setPower(intakePower);

        // Return current state
        return new WristAndIntakePower(wristPosition, intakePower);
    }
}
