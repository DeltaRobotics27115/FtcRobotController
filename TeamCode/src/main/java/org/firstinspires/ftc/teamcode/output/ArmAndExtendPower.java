package org.firstinspires.ftc.teamcode.output;

/**
 * A data class representing the power and target positions for the arm and extension mechanisms of a robot.
 */
public class ArmAndExtendPower {
    /**
     * The power value for the arm motor.
     */
    public double armPower;
    /**
     * The power value for the extension motor.
     */
    public double extendPower;
    /**
     * The target position for the arm.
     */
    public double armTargetPos;
    /**
     * The target position for the extension.
     */
    public double extendTargetPos;
    /**
     * The initial sensitivity for the arm.
     */
    public double sensitivityArmInit;
    /**
     * The initial sensitivity for the extension.
     */
    public double sensitivityExtendInit;
    /**
     * The scale factor for the arm.
     */
    public double scaleFactorArm;
    /**
     * The current position of the arm.
     */
    public int armPosition;
    /**
     * The current position of the extension.
     */
    public int extendPosition;

    /**
     * Creates a new ArmAndExtendPower object with the specified power values, target positions, sensitivities, scale factor, and current positions.
     *
     * @param ap                The power value for the arm motor.
     * @param ep                The power value for the extension motor.
     * @param armTargetPos      The target position for the arm.
     * @param extendTargetPos  The target position for the extension.
     * @param sensitivityArmInit The initial sensitivity for the arm.
     * @param sensitivityExtendInit The initial sensitivity for the extension.
     * @param scaleFactorArm    The scale factor for the arm.
     * @param armPosition       The current position of the arm.
     * @param extendPosition   The current position of the extension.
     */
    public ArmAndExtendPower(double ap, double ep, double armTargetPos, double extendTargetPos, double sensitivityArmInit, double sensitivityExtendInit, double scaleFactorArm, int armPosition, int extendPosition) {
        this.armPower = ap;
        this.extendPower = ep;
        this.armTargetPos = armTargetPos;
        this.extendTargetPos = extendTargetPos;
        this.sensitivityArmInit = sensitivityArmInit;
        this.sensitivityExtendInit = sensitivityExtendInit;
        this.scaleFactorArm = scaleFactorArm;
        this.armPosition = armPosition;
        this.extendPosition = extendPosition;
    }
}
