package org.firstinspires.ftc.teamcode.hardware;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.output.ArmAndExtendPower;

public class ArmExtentontrol {
    private PIDController armPID;
    private PIDController extendPID;

    private final DcMotorEx arm;
    private final DcMotorEx extend;

    private ElapsedTime armTimer;
    private ElapsedTime extendTimer;// Shared timer for both arm and extend

    public double armTargetPos = 0;
    public double extendTargetPos = 0;
    public double scaleFactorArm = 1.0;

    public double sensitivityExtendInit = 20.0;
    private final double maxTargetExtend = 5000; // Pre-calculate constant
    public double sensitivityArmInit = 20.0;
    public double ARM_TARGET_POS_HIGH;
    public double tick_in_degree =700/360;

    public double EXTEND_TARGET_POS_HIGH;
    public ArmExtentontrol(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize extend motor
        extend = hardwareMap.get(DcMotorEx.class, "Extend");
        extend.setDirection(DcMotor.Direction.REVERSE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize PID controllers with pre-defined constants
        armPID = new PIDController(0.00, 0.01, 0);
        extendPID = new PIDController(0.02, 0.04, 0);

    }
    public void init(double ARM_TARGET_POS_HIGH, double EXTEND_TARGET_POS_HIGH,double kP, double kI, double kD, double kF) {

        this.ARM_TARGET_POS_HIGH = ARM_TARGET_POS_HIGH;
        this.EXTEND_TARGET_POS_HIGH = EXTEND_TARGET_POS_HIGH;


    }
    public void setArmPID(double kP, double kI, double kD) {
        armPID.setPID(kP, kI, kD);

    }
    public void setExtendPID(double kP, double kI, double kD) {
        extendPID.setPID(kP, kI, kD);
    }
    public ArmAndExtendPower update(float right_stick_y, float left_stick_y, double armF, double extendF) {
        // Update extend target position
        extendTargetPos = Math.round(extendTargetPos - sensitivityExtendInit * right_stick_y);
        extendTargetPos = Math.min(extendTargetPos, EXTEND_TARGET_POS_HIGH); // Limit extend target position

        // Calculate arm sensitivity scale factor
        scaleFactorArm = 1 - (Math.abs(extendTargetPos) / maxTargetExtend);

        // Update arm target position
        armTargetPos = Math.round(armTargetPos - sensitivityArmInit * scaleFactorArm * left_stick_y);

        // Calculate motor powers using PID controllers
        double armPower = armPID.calculate( arm.getCurrentPosition(), armTargetPos);
        double extendPower = extendPID.calculate( extend.getCurrentPosition(), extendTargetPos);
        double armFf= Math.cos(Math.toRadians(armTargetPos/tick_in_degree))*armF;
        // Set motor powers
        arm.setPower(armPower+armFf);
        extend.setPower(extendPower);

        // Return current state
        return new ArmAndExtendPower(armPower, extendPower, armTargetPos, extendTargetPos,
                sensitivityArmInit, sensitivityExtendInit, scaleFactorArm,
                arm.getCurrentPosition(), extend.getCurrentPosition());
    }
}
