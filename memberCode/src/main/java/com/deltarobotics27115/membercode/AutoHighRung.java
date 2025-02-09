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
            if (time1 <= 400) {
                if (time1 <= 50) {
                    // arm up only p to move intake
                    error = 400 - arm.getCurrentPosition();
                    armPower = 0.01 * error;
                    arm.setPower(armPower);
                }
                if (time1 >= 50 && time1 <= 150) {
                    wrist.setPosition(0.5);
                }
                if (time1 >= 150) {
                    error = 0 - arm.getCurrentPosition();
                    armPower = 0.01 * error;
                    arm.setPower(armPower);
                }
                // left
                leftFrontValue = -1;
                leftBackValue = 1;
                rightFrontValue = 1;
                rightBackValue = -1;
            } else if (time1 <= 800) {
                // forward
                leftFrontValue = 1;
                leftBackValue = 1;
                rightFrontValue = 1;
                rightBackValue = 1;
            } else if (time1 <= 1200) {
                // arm up only p variable
                error = 1800 - arm.getCurrentPosition();
                armPower = 0.01 * error;
                arm.setPower(armPower);
            } else if (time1 <= 1600) {
                leftFrontValue = 0;
                leftBackValue = 0;
                rightBackValue = 0;
                rightFrontValue = 0;
                error = 1600 - arm.getCurrentPosition();
                armPower = 0.01 * error;
                arm.setPower(armPower);
            } else if (time1 <= 2000) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
                leftFrontValue = 0.2;
                leftBackValue = -1;
                rightFrontValue = -1;
                rightBackValue = 0.2;
                sleep(400);
                rightBackValue = -1;
                leftBackValue = 1;
                leftFrontValue = 1;
                sleep(600);
                intake.setPower(-1);
                sleep(1000);

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