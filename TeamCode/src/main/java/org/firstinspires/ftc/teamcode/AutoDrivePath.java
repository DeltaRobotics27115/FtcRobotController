package org.firstinspires.ftc.teamcode;

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
                leftFrontValue = 0.5;
                leftBackValue = -0.5;
                rightFrontValue = -0.5;
                rightBackValue = 0.5;
            }
            if (time > 1450 && time < 1550) {
                leftFrontValue = 0;
                leftBackValue = 0;
                rightFrontValue = 0;
                rightBackValue = 0;
            }
            //Go forward
            if (time > 1550 && time < 3550) {
                leftFrontValue = 0.5;
                leftBackValue = 0.5;
                rightFrontValue = 0.5;
                rightBackValue = 0.5;
            }
            if (time > 3550 && time < 3650) {
                leftFrontValue = 0;
                leftBackValue = 0;
                rightFrontValue = 0;
                rightBackValue = 0;
            }
            //Strafe right
            if (time > 3650 && time < 4100) {
                leftFrontValue = 0.5;
                rightFrontValue = -0.5;
                leftBackValue = -0.5;
                rightBackValue = 0.5;
            }
            if (time > 4100 && time < 4200) {
                leftFrontValue = 0;
                leftBackValue = 0;
                rightFrontValue = 0;
                rightBackValue = 0;
            }
            //Go backward
            if (time > 4200 && time < 6100) {
                leftFrontValue = -0.5;
                rightFrontValue = -0.5;
                leftBackValue = -0.5;
                rightBackValue = -0.5;
            }
            if (time > 6100 && time < 6200) {
                leftFrontValue = 0;
                leftBackValue = 0;
                rightFrontValue = 0;
                rightBackValue = 0;
            }
            //Go forward
            if (time > 6200 && time < 8200) {
                leftFrontValue = 0.5;
                leftBackValue = 0.5;
                rightFrontValue = 0.5;
                rightBackValue = 0.5;
            }
            if (time > 8200 && time < 8300) {
                leftFrontValue = 0;
                leftBackValue = 0;
                rightFrontValue = 0;
                rightBackValue = 0;
            }
            //Strafe right
            if (time > 8300 && time < 8700) {
                leftFrontValue = 0.5;
                rightFrontValue = -0.5;
                leftBackValue = -0.5;
                rightBackValue = 0.5;
            }
            if (time > 8700 && time < 8800) {
                leftFrontValue = 0;
                leftBackValue = 0;
                rightFrontValue = 0;
                rightBackValue = 0;
            }
            //Go backward
            if (time > 8800 && time < 10600) {
                leftFrontValue = -0.5;
                rightFrontValue = -0.5;
                leftBackValue = -0.5;
                rightBackValue = -0.5;
            }
            if (time > 10600 && time < 10700) {
                leftFrontValue = 0;
                leftBackValue = 0;
                rightFrontValue = 0;
                rightBackValue = 0;
            }
            frontLeft.setPower(leftFrontValue);
            backLeft.setPower(leftBackValue);
            frontRight.setPower(rightFrontValue);
            backRight.setPower(rightBackValue);
        }
    }
}

