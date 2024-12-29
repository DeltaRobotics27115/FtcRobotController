package com.deltarobotics27115.membercode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Autonomous extends LinearOpMode {
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
        //create variables for adjustment
        double leftFrontValue = 0, leftBackValue = 0, rightFrontValue = 0, rightBackValue = 0;
        //add timer
        ElapsedTime timer = new ElapsedTime();
        waitForStart();

        while (opModeIsActive()) {
            //count time
            double time = timer.milliseconds();
            double timex = 500;
            //Go forward until submersible
            if (time > 0 && time < timex) {
                leftFrontValue = 1;
                leftBackValue = 1;
                rightFrontValue = 1;
                rightBackValue = 1;
            }
            //Go right until wall
            if (time > timex && time < timex + 1500) {
                leftBackValue = -1;
                rightFrontValue = -1;
            }
            //Go left a little bit
            if (time > timex + 1500 && time < timex + 2000) {
                leftFrontValue = -1;
                leftBackValue = 1;
                rightFrontValue = 1;
                rightBackValue = -1;
            }
            //Go forward a little
            if (time > timex + 2000 && time < timex + 2500) {
                leftFrontValue = 1;
                rightBackValue = 1;
            }
            //Turn right then forward
            if (time > timex + 2500 && time < timex + 3000) {
                leftBackValue = 1;
                rightBackValue = -1;
            }
            //Strafe right
            if (time > timex + 3000 && time < timex + 3500) {
                rightFrontValue = -1;
                leftBackValue = -1;
                rightBackValue = 1;
            }
            //Strafe left
            if (time > timex + 3500 && time < timex + 3600) {
                rightFrontValue = 1;
                leftBackValue = 1;
                leftFrontValue = -1;
                rightBackValue = -1;
            }
            //Turn around
            if (time > timex + 3600 && time < timex + 3800) {
                rightFrontValue = 1;
                leftBackValue = -1;
                leftFrontValue = -1;
                rightBackValue = 1;
            }
            //Head to sample
            if (time > timex + 3800 && time < timex + 4500) {
                leftFrontValue = 1;
                rightFrontValue = 1;
                leftBackValue = 1;
                rightBackValue = 1;
            }
            frontLeft.setPower(leftFrontValue);
            backLeft.setPower(leftBackValue);
            frontRight.setPower(rightFrontValue);
            backRight.setPower(rightBackValue);
        }
    }
}
