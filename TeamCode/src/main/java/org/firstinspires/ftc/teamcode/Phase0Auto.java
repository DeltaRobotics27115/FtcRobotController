package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ArmAndExtendControl;
import org.firstinspires.ftc.teamcode.hardware.FieldCentricDriveControl;
import org.firstinspires.ftc.teamcode.hardware.WristAndIntakeControl;
import org.firstinspires.ftc.teamcode.output.ArmAndExtendPower;
import org.firstinspires.ftc.teamcode.util.ArmAndExtendState;

@Config
@Autonomous
public class Phase0Auto extends LinearOpMode {
    public static double kP=0.01;
    public static double kI=0.00;
    public static double kD=0.0;
    public static double kPextend=0.01;
    public static double kIextend=0.00;
    public static double kDextend=0.0;
    private WristAndIntakeControl wristAndIntake;
    private ArmAndExtendControl armAndExtend;
    public static double wristIncrement=0.02;
    private final double SLOW_MODE_TRIGGER_THRESHOLD = 0.3;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        wristAndIntake = new WristAndIntakeControl(hardwareMap);
        armAndExtend = new ArmAndExtendControl(hardwareMap);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart(); // Wait for the OpMode to start

        armAndExtend.initPosition();
        setArmPosition(1800,2000);
        setWristPosition(0.5,2000);

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
}
