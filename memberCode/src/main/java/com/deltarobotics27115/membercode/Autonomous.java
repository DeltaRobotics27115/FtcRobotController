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
            frontLeft.setPower(leftFrontValue);
            backLeft.setPower(leftBackValue);
            frontRight.setPower(rightFrontValue);
            backRight.setPower(rightBackValue);
            //Go forward until submersible
            //Go right until wall
            //Go left a little bit
            //Go forward a little
            //Turn right then forward
            //Strafe right
        }
    }
}