
package com.deltarobotics27115.membercode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricDRIVE extends LinearOpMode {
    //find motors
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    //find IMU
    BHI260IMU imu;
    @Override
    public void runOpMode() {
        //declare motors
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");
        //reverse direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        //wait for game to start
        waitForStart();
        double slowAmount = 0;
        //setup IMU
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        parameters.angleUnit = BHI260IMU.AngleUnit.DEGREES;
        parameters.mode = BHI260IMU.SensorMode.IMU;
        parameters.accelUnit = BHI260IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //declare IMU
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(parameters);
        //calculate IMU variables
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
        AxesOrder.ZYX, AngleUnit.DEGREES);
        initYaw = angles.firstAngle;
        //run until end
        while (opModeIsActive()) {
            //more IMU calculation
            angles=imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
            adjustedYaw = angles.firstAngle-initYaw;
            double zerodYaw = -initYaw+angles.firstAngle;
            //inputs
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            //angle calculation
            double theta = Math.atan2(y, x);
            double realTheta;
            realTheta = (360 - zerodYaw) + theta;
            double power = Math.hypot(x, y);
            double sin = Math.sin(realTheta - Math.PI/4);
            double cos = Math.cos(realTheta - Math.PI/4);
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
                rightFront /= power - turn;
                leftBack /= power + turn;
                rightBack /= power - turn;
            }
            //slow mode
            if (gamepad1.right_trigger > 0.1 && leftFront > 0.1) {
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


