package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.output.ArmAndExtendPower;

/**
 * This class controls the arm and extension mechanisms of the robot.
 * It uses PID controllers to achieve precise positioning.
 */
public class ArmAndExtendControl {

    private PIDController armPID;
    private  PIDController extendPID;

    private final DcMotorEx arm;
    private final DcMotorEx extend;

    public double armTargetPos = 0;
    public double extendTargetPos = 0;


    private  double initArmPosition=0;
    private double initExtendPosition=0;


    private  double maxArmPosition = 3000;

    private double maxExtendPosition =2050;



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
        extend.setDirection(DcMotor.Direction.REVERSE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize PID controllers with pre-defined constants
        armPID = new PIDController(0.01, 0.0, 0);
        extendPID = new PIDController(0.01, 0.0, 0);


    }
    public void setExtendMaxPosition(double position){
        this.maxExtendPosition=position;
    }
    public void setArmMaxPosition(double position){
        this.maxArmPosition=position;
    }
    public void setArmPID(double p, double i, double d) {
        armPID.setPID(p,i,d);
    }
    public void setExtendPID(double p, double i, double d) {
        extendPID.setPID(p,i,d);
    }


    /**
     * Updates the target positions and powers for the arm and extension motors.
     *
     * @param right_stick_y The vertical position of the right joystick.
     * @param left_stick_y  The vertical position of the left joystick.
     * @return An ArmAndExtendPower object containing the calculated power values and target positions.
     */
    public ArmAndExtendPower update(float right_stick_y, float left_stick_y,double perDistcnce) {
        // change power of arm and extend
        if(left_stick_y!=0){
            armTargetPos=arm.getCurrentPosition()+left_stick_y*perDistcnce;
            armTargetPos=clamp(armTargetPos,this.initArmPosition,maxArmPosition);

        }


        if(right_stick_y!=0){
            extendTargetPos=extend.getCurrentPosition()+right_stick_y*perDistcnce;
            extendTargetPos=clamp(extendTargetPos,this.initExtendPosition,maxExtendPosition);

        }


        double pidArm=armPID.calculate(arm.getCurrentPosition(),armTargetPos);
        //double ff=Math.cos(Math.toRadians(armTargetPos/tick_in_degree))*f;
        double pidExtend=extendPID.calculate(extend.getCurrentPosition(),extendTargetPos);
        arm.setPower(pidArm);
        extend.setPower(pidExtend);


        // Return current state
        return new ArmAndExtendPower(pidArm, pidExtend, armTargetPos, extendTargetPos,

                arm.getCurrentPosition(), extend.getCurrentPosition());
    }
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
    public void initPosition(){
            this.initArmPosition=arm.getCurrentPosition();
        this.initExtendPosition=extend.getCurrentPosition();
    }
    public void setToStartPosition(){
        double tarP=this.initArmPosition+800;
        double armPower = armPID.calculate(arm.getCurrentPosition(),tarP);
            arm.setPower(armPower);
    }
    public void setArmDown(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public ArmAndExtendPower toCatchPosition() {
        double armPower = armPID.calculate(arm.getCurrentPosition(),this.initArmPosition);
        double extendPower = extendPID.calculate(extend.getCurrentPosition(),maxExtendPosition);

        arm.setPower(armPower);
        extend.setPower(extendPower);
        return new ArmAndExtendPower(armPower, extendPower, armTargetPos, extendTargetPos,

                arm.getCurrentPosition(), extend.getCurrentPosition());
    }
    public double getArmPosition(){
        return arm.getCurrentPosition();
    }

    public double getExtendPosition(){
        return extend.getCurrentPosition();
    }
    public ArmAndExtendPower toMovePosition() {
        double armPower = armPID.calculate( arm.getCurrentPosition(),initArmPosition);


        double extendPower = extendPID.calculate(extend.getCurrentPosition(),initExtendPosition);
        armPower = arm.isOverCurrent() ? 0 : armPower;
        extendPower = extend.isOverCurrent() ? 0 : extendPower;
        double ff=Math.abs(extend.getCurrentPosition()-initExtendPosition);
        extend.setPower(extendPower);
        if(ff<100) {

            arm.setPower(armPower);
        }


        return new ArmAndExtendPower(armPower, extendPower, armTargetPos, extendTargetPos,

                arm.getCurrentPosition(), extend.getCurrentPosition());
    }
    public ArmAndExtendPower toShootPosition(double armShootPosition,double extendShootPosition) {
        // Cache current arm position
        double currentArmPosition = arm.getCurrentPosition();

        double armPower = armPID.calculate(currentArmPosition, armShootPosition);
        double extendPower = extendPID.calculate(extend.getCurrentPosition(), extendShootPosition);

        arm.setPower(armPower);

        // Simplified conditional check: If arm is within 180 of the target
        if (Math.abs(armShootPosition - currentArmPosition) <= armShootPosition/10) {
            extend.setPower(extendPower);
        }

        return new ArmAndExtendPower(armPower, extendPower, armTargetPos, extendTargetPos,
                arm.getCurrentPosition(), extend.getCurrentPosition());
    }
    public void setArmPosition(double position){
        double armPower = armPID.calculate(arm.getCurrentPosition(),position);
        arm.setPower(armPower);
    }
}