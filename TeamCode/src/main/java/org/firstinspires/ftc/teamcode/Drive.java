package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Drive extends LinearOpMode {
    //find motors
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    @Override
    public void runOpMode() {
        //declare motors
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");
        //wait for game to start
        waitForStart();
        double slowAmount = 0;
        //reverse direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        //run until end
        while (opModeIsActive()) {
            //inputs
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            //angle calculation
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);
            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.max(Math.abs(sin),
                    Math.abs(cos));
            //power calculation
            double leftFront = power * cos/max + turn;
            double rightFront = power * sin/max - turn;
            double leftBack = power * sin/max + turn;
            double rightBack = power * cos/max - turn;
            //cut power to 1
            if ((power + Math.abs(turn)) > 1) {
                leftFront /= power + turn;
                rightFront /= power + turn;
                leftBack /= power + turn;
                rightBack /= power + turn;
            }
            //slow mode
            if (gamepad1.right_trigger > 0.1) {
                slowAmount = 0.3;
            } else {
                slowAmount = 0;
            }
            //run
            frontLeft.setPower(leftFront - slowAmount);
            frontRight.setPower(rightFront - slowAmount);
            backLeft.setPower(leftBack - slowAmount);
            backRight.setPower(rightBack - slowAmount);
        }
    }
}