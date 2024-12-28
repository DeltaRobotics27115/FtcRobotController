package com.deltarobotics27115.membercode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Autonomous extends LinearOpMode {
    //find motors
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        //declare motors
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");
        //create variables for adjustment
        double leftFrontValue = 0, leftBackValue = 0, rightFrontValue = 0, rightBackValue = 0;
        //add timer
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) {
            //count time
            double time = timer.milliseconds();
            //Go forward until submersible
            if (time > 0 && time < 500) {
                leftFrontValue = 1;
                leftBackValue = 1;
                rightFrontValue = 1;
                rightBackValue = 1;
            }
            //Go right until wall
            if (time > 500 && time < 2000) {
                leftBackValue = -1;
                rightFrontValue = -1;
            }
            //Go left a little bit
            if (time > 2000 && time < 2500) {
                leftFrontValue = -1;
                leftBackValue = 1;
                rightFrontValue = 1;
                rightBackValue = -1;
            }
            //*******DANIEL CODE THIS PART******
            //Go forward a little
            //Turn right then forward
            //Strafe right
            frontLeft.setPower(leftFrontValue);
            backLeft.setPower(leftBackValue);
            frontRight.setPower(rightFrontValue);
            backRight.setPower(rightBackValue);
        }
    }
}
