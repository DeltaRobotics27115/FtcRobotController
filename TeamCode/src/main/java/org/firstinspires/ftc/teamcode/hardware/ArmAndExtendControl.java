package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.output.ArmAndExtendPower;

public class ArmAndExtendControl {
    private final PIDController armPID;
    private final PIDController extendPID;
    // ... (other variables)
    private final DcMotorEx arm;
    private final DcMotorEx extend;

    private ElapsedTime armTimer;
    private ElapsedTime extendTimer;

    //private double lastErrorArm = 0;
    // Integral error for arm PID
    //private double errorSumArm = 0;
    // Elapsed time for arm PID
    public double armTargetPos = 0; // adjust
    // Extend sensitivity
    public double sensitivityExtendInit = 5.0; // adjust
    public double extendTargetPos = 0; // adjust

    public ArmAndExtendControl(HardwareMap hardwareMap) {
        // ... (motor initialization)
        arm = hardwareMap.get(DcMotorEx.class, "Arm"); // change device name
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setPower(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extend = hardwareMap.get(DcMotorEx.class, "Extend"); // change device name
        extend.setDirection(DcMotor.Direction.FORWARD);
        extend.setPower(0);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize PID controllers
        // adjust
        double kPArm = 0.02;
        // adjust
        double kIArm = 0;
        // adjust
        double kDArm = 0;
        armPID = new PIDController(kPArm, kIArm, kDArm);
        // adjust
        double kPExtend = 0.01;
        // adjust
        double kIExtend = 0;
        // adjust
        double kDExtend = 0;
        extendPID = new PIDController(kPExtend, kIExtend, kDExtend);

    }

    public void startArmTimer() {
        armTimer = new ElapsedTime();
        extendTimer = new ElapsedTime();
    }

    public ArmAndExtendPower update(float right_stick_y, float left_stick_y) {
        // ... (update target positions based on gamepad input or other logic)
        extendTargetPos = Math.round(extendTargetPos - sensitivityExtendInit * right_stick_y);
        if (extendTargetPos > 450) extendTargetPos = 450;
        // Arm sensitivity scale factor associated to the current extend status
        //private double lastErrorExtend = 0;
        // Integral error for extend PID
        //private double errorSumExtend = 0;
        // adjust
        double maxTargetExtend = 5000;
        double scaleFactorArm = 1 - (Math.abs(extendTargetPos) / maxTargetExtend);
        // Arm initial sensitivity
        // adjust
        double sensitivityArmInit = 5.0;
        armTargetPos = Math.round(armTargetPos - sensitivityArmInit * scaleFactorArm * left_stick_y);
        // Use the generic PID controller
        double armPower = armPID.calculate(armTargetPos, arm.getCurrentPosition(), armTimer);
        arm.setPower(armPower);
        double extendPower = extendPID.calculate(extendTargetPos, extend.getCurrentPosition(), extendTimer);
        extend.setPower(extendPower);
        return new ArmAndExtendPower(armPower, extendPower, armTargetPos, extendTargetPos, sensitivityArmInit, sensitivityExtendInit, scaleFactorArm, arm.getCurrentPosition(), extend.getCurrentPosition());
    }

}
