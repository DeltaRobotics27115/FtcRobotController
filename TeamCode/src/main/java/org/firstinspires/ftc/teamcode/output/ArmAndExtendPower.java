package org.firstinspires.ftc.teamcode.output;

/**
 * A data class representing the power and target positions for the arm and extension mechanisms of a robot.
 */
public class ArmAndExtendPower {
    /**
     * The power value for the arm motor.
     */
    public double armPower=0;
    /**
     * The power value for the extension motor.
     */
    public double extendPower=0;
    /**
     * The target position for the arm.
     */
    public double armTargetPos=0;
    /**
     * The target position for the extension.
     */
    public double extendTargetPos=0;


    /**
     * The current position of the arm.
     */
    public int armPosition=0;
    /**
     * The current position of the extension.
     */
    public int extendPosition=0;

    /**
     * Creates a new ArmAndExtendPower object with the specified power values, target positions, sensitivities, scale factor, and current positions.
     *
     * @param ap                The power value for the arm motor.
     * @param ep                The power value for the extension motor.
     * @param armTargetPos      The target position for the arm.
     * @param extendTargetPos  The target position for the extension.
     * @param armPosition       The current position of the arm.
     * @param extendPosition   The current position of the extension.
     */
    public ArmAndExtendPower(double ap, double ep, double armTargetPos, double extendTargetPos, int armPosition, int extendPosition) {
        this.armPower = ap;
        this.extendPower = ep;
        this.armTargetPos = armTargetPos;
        this.extendTargetPos = extendTargetPos;

        this.armPosition = armPosition;
        this.extendPosition = extendPosition;
    }
}
