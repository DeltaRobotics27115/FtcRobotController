package org.firstinspires.ftc.teamcode.output;

/**
 * A class representing the power values for each motor of a four-wheel drive robot.
 */
public class DrivePower {
    /**
     * The power value for the left front motor.
     */
    public double leftFront;
    /**
     * The power value for the right front motor.
     */
    public double rightFront;
    /**
     * The power value for the left back motor.
     */
    public double leftBack;
    /**
     * The power value for the right back motor.
     */
    public double rightBack;

    /**
     * Creates a new DrivePower object with the specified power values for each motor.
     *
     * @param lf The power value for the left front motor.
     * @param rf The power value for the right front motor.
     * @param lb The power value for the left back motor.
     * @param rb The power value for the right back motor.
     */
    public DrivePower(double lf, double rf, double lb, double rb) {
        leftFront = lf;
        rightFront = rf;
        leftBack = lb;
        rightBack = rb;

    }


}