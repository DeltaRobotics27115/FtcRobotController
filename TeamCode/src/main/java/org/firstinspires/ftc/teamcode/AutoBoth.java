package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ArmAndExtendControl;
import org.firstinspires.ftc.teamcode.hardware.WristAndIntakeControl;

@Config
@Autonomous
public class AutoBoth extends LinearOpMode {
    public static double kP=0.01;
    public static double kI=0.00;
    public static double kD=0.0;
    public static double kPextend=0.01;
    public static double kIextend=0.00;
    public static double kDextend=0.0;
    private WristAndIntakeControl wristAndIntake;
    private ArmAndExtendControl armAndExtend;
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() {
        wristAndIntake = new WristAndIntakeControl(hardwareMap);
        armAndExtend = new ArmAndExtendControl(hardwareMap);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        double leftFrontValue = 0, leftBackValue = 0, rightFrontValue = 0, rightBackValue = 0;
        //add timer
        ElapsedTime timer = new ElapsedTime();
        waitForStart(); // Wait for the OpMode to start

        armAndExtend.initPosition();
        setArmPosition(1800,2000);
        setWristPosition(0.5,2000);
        driveBasePath(leftFrontValue, leftBackValue, rightFrontValue, rightBackValue);

    }
    public void setWristPosition(double wristPosition, double timeOut){
        if(opModeIsActive()){
            runtime.reset();
        }
        double wristCurrentPosition=wristAndIntake.getWristPosition();
        while (opModeIsActive() && runtime.milliseconds()<timeOut|| wristCurrentPosition!=wristPosition){
            wristAndIntake.setWristPosition(0.5);
            wristCurrentPosition=wristAndIntake.getWristPosition();
        }
    }
    public void setArmPosition(double armPosition, double timeOut) {
        if(opModeIsActive()){
            runtime.reset();
        }
        while(opModeIsActive() && runtime.milliseconds()<timeOut){
            armAndExtend.setArmPosition(armPosition);
        }

    }
    public void driveBasePath(double leftFrontValue, double leftBackValue, double rightFrontValue, double rightBackValue){
        if(opModeIsActive()){
            timer.reset();
        }
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
