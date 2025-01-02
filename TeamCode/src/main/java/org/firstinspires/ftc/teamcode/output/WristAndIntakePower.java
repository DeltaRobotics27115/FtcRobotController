package org.firstinspires.ftc.teamcode.output;

/**
 * This class represents the power levels for the wrist and intake mechanisms.
 */
public class WristAndIntakePower {
    /**
     * The power level for the wrist servo (typically between 0.0 and 1.0).
     */
    public double wristPosition;
    /**
     * The power level for the intake motor (typically between -1.0 and 1.0).
     */
    public double intakePower;
    public double currentWristPosition;



    public WristAndIntakePower(double wristPosition, double intake, double currentWiristPosition) {
        wristPosition = wristPosition;
        intakePower = intake;
        currentWristPosition = currentWiristPosition;

    }
}
