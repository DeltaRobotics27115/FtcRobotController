package org.firstinspires.ftc.teamcode.output;

public class ArmAndExtendPower {
    public double armPower;
    public double extendPower;
    public double armTargetPos;
    public double extendTargetPos;
    public double sensitivityArmInit;
    public double sensitivityExtendInit;
    public double scaleFactorArm;
    public int armPosition;
    public int extendPosition;

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
