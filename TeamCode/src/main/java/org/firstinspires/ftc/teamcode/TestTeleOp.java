package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp
public class TestTeleOp extends LinearOpMode {
    public static double kP=0.03;
    public static double kI=0.03;
    public static double kD=0.0;
    public static double f=0.6;
    public static double targetPosition=0;
    public static double tick_in_degree=3.0;

    public static double fExtend=0.0;


    private DcMotorEx arm;
    private PIDController pid;
    @Override
    public void runOpMode() throws InterruptedException {

        arm= hardwareMap.get(DcMotorEx.class,"Arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pid=new PIDController(kP,kI,kD);
        waitForStart();
        while (opModeIsActive()) {
            pid.setPID(kP,kI,kD);


            int armPosition=arm.getCurrentPosition();

            double pidP=pid.calculate(arm.getCurrentPosition(),targetPosition);
            double ff=Math.cos(Math.toRadians(targetPosition/tick_in_degree))*f;



            arm.setPower(pidP+ff);


            telemetry.addData("ArmPosition",armPosition);
            telemetry.addData("TargetPosition",targetPosition);


            telemetry.update();
        }


        }



}
