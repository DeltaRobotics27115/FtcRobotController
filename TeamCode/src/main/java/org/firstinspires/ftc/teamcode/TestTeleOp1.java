package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
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
public class TestTeleOp1 extends LinearOpMode {
    private DcMotorEx extend;
    public static int targetPosition=0;
    public static double kP=0.03;
    public static double kI=0.03;
    public static double kD=0.0;

    private  PIDController pid;
    public static double positionCoefficient=0.05;
    public static double tolerance=13.6;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        extend=hardwareMap.get(DcMotorEx.class,"Extend");
        extend.setDirection(DcMotorSimple.Direction.REVERSE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid=new PIDController(kP,kI,kD);
        waitForStart();
        while (opModeIsActive()) {
            pid.setPID(kP,kI,kD);
            double pidP=pid.calculate(extend.getCurrentPosition(),targetPosition);
            telemetry.addData("pidp",pidP);
           extend.setPower(pidP);

            telemetry.addData("Position", extend.getCurrentPosition());
            telemetry.addData("Target", targetPosition);
            telemetry.update();

        }
    }
}
