package org.firstinspires.ftc.teamcode.output;

/**
 * This class represents the power levels for the wrist and intake mechanisms.
 */
public class WristAndIntakePower {
    /**
     * The power level for the wrist servo (typically between 0.0 and 1.0).
     */
    public double wristPower;
    /**
     * The power level for the intake motor (typically between -1.0 and 1.0).
     */
    public double intakePower;

    /**
     * Constructor for the WristAndIntakePower class.
     *
     * @param wrist  The power level for the wrist servo.
     * @param intake The power level for the intake motor.
     */
    public WristAndIntakePower(double wrist, double intake) {
        wristPower = wrist;
        intakePower = intake;
    }
}
