package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.output.WristAndIntakePower;

/**
 * This class controls the wrist and intake mechanisms of the robot.
 */
public class WristAndIntakeControl {

    private   double WRIST_INCREMENT = 0.02; // Define increment as a constant
    private static final double TRIGGER_THRESHOLD = 0.2; // Define trigger threshold as a constant

    private final Servo wrist;
    private final Servo intake;

    private double wristPosition = 0;
    private double intakePosition = 0; // Store intake power directly

    /**
     * Constructor for WristAndIntakeControl.
     * Initializes the wrist and intake hardware.
     *
     * @param hardwareMap The hardware map to access the wrist and intake.
     */
    public WristAndIntakeControl(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "Wrist");
        intake = hardwareMap.get(Servo.class, "Intake");
        wrist.setDirection(Servo.Direction.REVERSE);
        //wrist.setPosition(0.5);
    }
    public void setWristIncrement(double increment){
        this.WRIST_INCREMENT=increment;
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
        wrist.setPosition(wristPosition);

        // Update intake power based on bumpers
        if (leftBumper) {
            intakePosition = 0.5;
        } else if (rightBumper) {
            intakePosition = -1;
        }
        intake.setPosition(intakePosition);


        // Return current state
        return new WristAndIntakePower(wristPosition, intakePosition,wrist.getPosition());
    }
    public double getWristPosition() {
        return wrist.getPosition();
    }

    public void setWristPosition(double wristPosition) {
        wrist.setPosition(wristPosition);
    }
}
