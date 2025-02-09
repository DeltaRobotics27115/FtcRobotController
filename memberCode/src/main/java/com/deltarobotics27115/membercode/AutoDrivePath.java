package com.deltarobotics27115.membercode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutoDrivePath extends LinearOpMode {
    //find motors
    public DcMotor frontLeft, frontRight, backLeft, backRight, arm;

    @Override
    public void runOpMode() {
        //declare motors
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");
        arm = hardwareMap.get(DcMotor.class, "Arm");
        //reverse motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //create variables
        double leftFrontValue = 0, leftBackValue = 0, rightFrontValue = 0, rightBackValue = 0;
        //add timer
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) {
            //count time
            double time = timer.milliseconds();
            //Side 900ms
            //Forward 1000ms
            //Go right
            if (time > 1000 && time < 1450) {
                leftFrontValue = -0.5;
                leftBackValue = 0.5;
                rightFrontValue = 0.5;
                rightBackValue = -0.5;
            }
            if (time > 1450 && time < 1950) {
                leftFrontValue = -0.5;
                leftBackValue = -0.5;
                rightFrontValue = 0.5;
                rightBackValue = 0.5;
            }
            //EXTEND AND DROP SAMPLE 4 SEC
            if (time > 5950 && time < 5450) {
                leftFrontValue = 0.5;
                leftBackValue = 0.5;
                rightFrontValue = -0.5;
                rightBackValue = -0.5;
            }
            //EXTEND AND GRAB SAMPLE 5 SEC
            if (time > 10450 && time < 10950) {
                leftFrontValue = -0.5;
                leftBackValue = -0.5;
                rightFrontValue = 0.5;
                rightBackValue = 0.5;
            }
            //EXTEND AND DROP SAMPLE 4 SEC
            if (time > 14950 && time < 15450) {
                leftFrontValue = 0.5;
                leftBackValue = 0.5;
                rightFrontValue = -0.5;
                rightBackValue = -0.5;
            }
            if (time > 15450 && time < 17450) {
                leftFrontValue = 0.5;
                leftBackValue = 0.5;
                rightFrontValue = 0.5;
                rightBackValue = 0.5;
            }
            if (time > 17450 && time < 17675) {
                leftFrontValue = 0.5;
                leftBackValue = -0.5;
                rightFrontValue = -0.5;
                rightBackValue = 0.5;
            }
            if (time > 17675 && time < 1800) {

            }
            frontLeft.setPower(leftFrontValue);
            backLeft.setPower(leftBackValue);
            frontRight.setPower(rightFrontValue);
            backRight.setPower(rightBackValue);
        }
    }
}