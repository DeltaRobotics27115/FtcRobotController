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
public class AutoTest extends LinearOpMode {
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
            //Turn 90. 500ms
            //Go forward
            if (time > 100 && time < 600) {
                leftFrontValue = 0.5;
                leftBackValue = 0.5;
                rightFrontValue = 0.5;
                rightBackValue = 0.5;
            }
            if (time > 600 && time < 700) {
                leftFrontValue = 0;
                leftBackValue = 0;
                rightFrontValue = 0;
                rightBackValue = 0;
            }
            //Strafe left
            if (time > 700 && time < 1600) {
                leftFrontValue = -0.5;
                rightFrontValue = 0.5;
                leftBackValue = 0.5;
                rightBackValue = -0.5;
            }
            if (time > 1600 && time < 1700) {
                leftFrontValue = 0;
                leftBackValue = 0;
                rightFrontValue = 0;
                rightBackValue = 0;
            }
            //Turn 135.
            if (time > 1700 && time < 2450) {
                leftFrontValue = -0.5;
                rightFrontValue = -0.5;
                leftBackValue = -0.5;
                rightBackValue = -0.5;
            }
            if (time > 2450 && time < 2550) {
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
