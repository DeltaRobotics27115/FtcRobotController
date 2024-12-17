package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.output.ArmAndExtendPower;

/**
 * This class controls the arm and extension mechanisms of the robot.
 * It uses PID controllers to achieve precise positioning.
 */
public class ArmAndExtendControl {

    private final PIDController armPID;
    private final PIDController extendPID;

    private final DcMotorEx arm;
    private final DcMotorEx extend;

    private ElapsedTime armTimer;
    private ElapsedTime extendTimer;// Shared timer for both arm and extend

    public double armTargetPos = 0;
    public double extendTargetPos = 0;

    public double sensitivityExtendInit = 5.0;
    private final double maxTargetExtend = 5000; // Pre-calculate constant
    public double sensitivityArmInit = 5.0;

    /**
     * Constructor for ArmAndExtendControl.
     * Initializes the motors and PID controllers.
     *
     * @param hardwareMap The hardware map to access the motors.
     */
    public ArmAndExtendControl(HardwareMap hardwareMap) {
        // Initialize arm motor
        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize extend motor
        extend = hardwareMap.get(DcMotorEx.class, "Extend");
        extend.setDirection(DcMotor.Direction.FORWARD);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize PID controllers with pre-defined constants
        armPID = new PIDController(0.02, 0, 0);
        extendPID = new PIDController(0.01, 0, 0);


    }
    public void startTimer() {
        armTimer = new ElapsedTime();
        extendTimer = new ElapsedTime();
    }

    /**
     * Updates the target positions and powers for the arm and extension motors.
     *
     * @param right_stick_y The vertical position of the right joystick.
     * @param left_stick_y  The vertical position of the left joystick.
     * @return An ArmAndExtendPower object containing the calculated power values and target positions.
     */
    public ArmAndExtendPower update(float right_stick_y, float left_stick_y) {
        // Update extend target position
        extendTargetPos = Math.round(extendTargetPos - sensitivityExtendInit * right_stick_y);
        extendTargetPos = Math.min(extendTargetPos, 450); // Limit extend target position

        // Calculate arm sensitivity scale factor
        double scaleFactorArm = 1 - (Math.abs(extendTargetPos) / maxTargetExtend);

        // Update arm target position
        armTargetPos = Math.round(armTargetPos - sensitivityArmInit * scaleFactorArm * left_stick_y);

        // Calculate motor powers using PID controllers
        double armPower = armPID.calculate(armTargetPos, arm.getCurrentPosition(), armTimer);
        double extendPower = extendPID.calculate(extendTargetPos, extend.getCurrentPosition(), extendTimer);

        // Set motor powers
        arm.setPower(armPower);
        extend.setPower(extendPower);

        // Return current state
        return new ArmAndExtendPower(armPower, extendPower, armTargetPos, extendTargetPos,
                sensitivityArmInit, sensitivityExtendInit, scaleFactorArm,
                arm.getCurrentPosition(), extend.getCurrentPosition());
    }
}