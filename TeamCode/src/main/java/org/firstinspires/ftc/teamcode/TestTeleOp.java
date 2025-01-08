package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.FieldCentricDriveControl;

@Config
@Autonomous
public class TestTeleOp extends LinearOpMode {
   public static double x=0;
   public static double y=0.5;
   public static double turn=0;
   public static double timeOut=1;
   public static double slowMode=0.7;
    private FieldCentricDriveControl fieldCentricDrive;
    public static boolean start=true;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        fieldCentricDrive = new FieldCentricDriveControl(hardwareMap);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        fieldCentricDrive.resetImu();
        if(start){
            drive(x,y,turn,timeOut);
            double  heading=fieldCentricDrive.getHeading();
            telemetry.addData("heading",heading);
        }
        while(opModeIsActive()){
            telemetry.update();
        }
        }
    public void drive(double x, double y, double turn, double timeOut) {
        if(opModeIsActive()){
            runtime.reset();

        }
        while(opModeIsActive() && runtime.milliseconds()<timeOut){
            fieldCentricDrive.driveFieldCentric(x, y, turn, slowMode);
        }
        start=false;

    }




}
