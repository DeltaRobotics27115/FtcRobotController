package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp()
public class PIDArm extends LinearOpMode {
    private DcMotorEx arm;
    private DcMotorEx extend;
    private final double kP1 = 0.02; // adjust
    private final double kI1 = 0; // adjust
    private final double kD1 = 0; // adjust
    private final double kP2 = 0.01; // adjust
    private final double kI2 = 0; // adjust
    private final double kD2 = 0; // adjust
    private static double target1 = -120; // adjust
    private static double target2 = 0; // adjust
    private double sensitivityExtend = 2.0; // adjust
    private double sensitivityArmInit = 2.0; // adjust
    private double scaleFactorArm = 1;
    private double maxTarget2 = 5000; // adjust

    private double lastError1 = 0;
    private double errorSum1 = 0;
    private ElapsedTime timer1;
    private ElapsedTime timer2;

    @Override
    public void runOpMode() throws InterruptedException {
        initArm();
        initExtend();
        armAndExtendTelemetry();
        waitForStart();
        timer1 = new ElapsedTime();
        timer2 = new ElapsedTime();

        while (opModeIsActive()) {
            target2 = Math.round(target2 - sensitivityExtend * gamepad1.right_stick_y);
            scaleFactorArm = 1 - (Math.abs(target2) / maxTarget2);
            target1 = Math.round(target1 - sensitivityArmInit * scaleFactorArm * gamepad2.left_stick_y);
            double power1 = pidControl(target1, arm.getCurrentPosition(), kP1, kI1, kD1);
            double power2 = pidControl(target2, extend.getCurrentPosition(), kP2, kI2, kD2);
            arm.setPower(power1);
            extend.setPower(power2);
            armAndExtendTelemetry();
        }
    }


    public void initArm() {
        arm = hardwareMap.get(DcMotorEx.class, "Arm"); // change device name
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setPower(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initExtend() {
        extend = hardwareMap.get(DcMotorEx.class, "Extend"); // change device name
        extend.setDirection(DcMotor.Direction.REVERSE);
        extend.setPower(0);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double pidControl(double target, double curPos, double kP1, double kI1, double kD1) {
        double error1 = target - curPos;
        errorSum1 += error1 * timer1.seconds();
        double errorRate1 = (error1 - lastError1) / timer1.seconds();
        lastError1 = error1;
        timer1.reset();
        return kP1 * error1 + kI1 * errorSum1 + kD1 * errorRate1;
    }

    public void armAndExtendTelemetry() {
        telemetry.addData("Target For Arm", target1);
        telemetry.addData("Current Position Of Arm", arm.getCurrentPosition());
        telemetry.addData("Sensivity Of Arm", sensitivityArmInit * scaleFactorArm);
        telemetry.addData("Target For Extend", target2);
        telemetry.addData("Current Position Of Extend", extend.getCurrentPosition());
        telemetry.addData("Sensivity Of Extend", sensitivityExtend);
        telemetry.update();
    }
}