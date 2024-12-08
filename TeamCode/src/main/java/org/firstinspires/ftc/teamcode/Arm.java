package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp()
public class Arm extends LinearOpMode {
    // ****** Arm variables ******
    // Arm motor
    private DcMotorEx arm;
    // arm power
    private double armPower;
    // PID coefficients for arm
    private final double kPArm = 0.02; // adjust
    private final double kIArm = 0; // adjust
    private final double kDArm = 0; // adjust
    // Last error for arm PID
    private double lastErrorArm = 0;
    // Integral error for arm PID
    private double errorSumArm = 0;
    // Elapsed time for arm PID
    private ElapsedTime armTimer;
    // Arm target position
    private static double armTargetPos = -120; // adjust
    // Arm initial sensitivity
    private double sensitivityArmInit = 2.0; // adjust
    // Arm sensitivity scale factor associated to the current extend status
    private double scaleFactorArm = 1;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // init arm
        initArm();
        // telemetry
        armTelemetry();

        waitForStart();
        // start timer
        armTimer = new ElapsedTime();

        // TeleOp loop
        while (opModeIsActive()) {
            // use buttons to set target positions
            // arm buttons
            if (gamepad2.a) {
                armTargetPos = -120; // adjust
            } else if (gamepad2.y) {
                armTargetPos = 2000; // adjust
            }

            // move arm with joysticks
            scaleFactorArm = 1 - (Math.abs(extendTargetPos) / maxTargetExtend);
            armTargetPos = Math.round(armTargetPos - sensitivityArmInit * scaleFactorArm * gamepad2.left_stick_y);

            // set motor power with PID
            armPower = armControl(armTargetPos, arm.getCurrentPosition(), kPArm, kIArm, kDArm);
            arm.setPower(armPower);
            
            // telemetry
            armTelemetry();
        }
    }

    // function to init arm
    public void initArm() {
        arm = hardwareMap.get(DcMotorEx.class, "Arm"); // change device name
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setPower(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // function for pid control for arm
    public double armControl(double target, double curPos, double kP1, double kI1, double kD1) {
        // error
        double error1 = target - curPos;
        // integral error
        errorSumArm += error1 * armTimer.seconds();
        // derivative
        double errorRate1 = (error1 - lastErrorArm) / armTimer.seconds();
        // reset last error
        lastErrorArm = error1;
        // reset timer
        armTimer.reset();
        // return power
        return kP1 * error1 + kI1 * errorSumArm + kD1 * errorRate1;
    }

    // function to display motor status
    public void armTelemetry() {
        // telemetry
        telemetry.addData("Target For Arm", armTargetPos);
        telemetry.addData("Current Position Of Arm", arm.getCurrentPosition());
        telemetry.addData("Sensitivity Of Arm", sensitivityArmInit * scaleFactorArm);
        telemetry.update();
    }
}
