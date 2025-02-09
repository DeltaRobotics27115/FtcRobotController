package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous
public class AutoDrive extends LinearOpMode {
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public CRServo intake;
    public DcMotorEx extend, arm;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intake = hardwareMap.get(CRServo.class, "Intake");
        extend = hardwareMap.get(DcMotorEx.class, "Extend");
        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


        }

    }
}