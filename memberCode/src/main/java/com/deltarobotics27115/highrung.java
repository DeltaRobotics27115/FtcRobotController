package com.deltarobotics27115.membercode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutoHighRung extends LinearOpMode{
    public CRServo intake;
    public Servo wrist;
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotorEx arm, extend;
    public double leftFrontValue = 0, leftBackValue = 0, rightFrontValue = 0, rightBackValue = 0;
    public double error, armPower;
    @Override
    public void runOpMode() {
        initAllHardware();
        ElapsedTime timer1 = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) {
            double time1 = timer1.milliseconds();
            if (time1 <= 200) {
                if (time1 <= 50) {
                    error = 400 - arm.getCurrentPosition();
                    armPower = 0.01 * error;
                    arm.setPower(armPower);
                }
                if (time1 >= 50 && time1 <= 150) {
                    wrist.setPosition(0.5);
                }
                if (time1 >= 150) {
                    error = -arm.getCurrentPosition();
                    armPower = 0.01 * error;
                    arm.setPower(armPower);
                }
                leftFrontValue = -1;
                leftBackValue = 1;
                rightFrontValue = 1;
                rightBackValue = -1;
            } else if (time1 <= 400) {
                leftFrontValue = 1;
                leftBackValue = 1;
                rightFrontValue = 1;
                rightBackValue = 1;
            } else if (time1 <= 600) {
                error = 2000 - arm.getCurrentPosition();
                armPower = 0.01 * error;
                arm.setPower(armPower);
            } else if (time1 <= 800) {
                // arm down only p variable
                leftFrontValue = 0;
                leftBackValue = 0;
                rightFrontValue = 0;
                rightBackValue = 0;
                error = 1600 - arm.getCurrentPosition();
                armPower = 0.01 * error;
                arm.setPower(armPower);
            } else if (time1 <= 900) {

            }
        }
    }

    public void initAllHardware() {
        intake = hardwareMap.get(CRServo.class("Intake"));
        wrist = hardwareMap.get(Servo.class("Wrist"));
        frontLeft = hardwareMap.get(DcMotor.class("frontLeft"));
        frontRight = hardwareMap.get(DcMotor.class("frontRight"));
        backLeft = hardwareMap.get(DcMotor.class("backLeft"));
        backRight = hardwareMap.get(DcMotor.class("backRight"));
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        arm = hardwareMap.get(DcMotorEx.class("Arm"));
        extend = hardwareMap.get(DcMotorEx.class("Extend"));
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
