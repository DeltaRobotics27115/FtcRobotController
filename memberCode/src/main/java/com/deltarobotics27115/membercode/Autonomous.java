package com.deltarobotics27115.membercode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        //Go forward until submersible
        //Go right until wall
        //Go left a little bit
        //Go forward a little
        //Turn right then forward
        //Strafe right
    }
}