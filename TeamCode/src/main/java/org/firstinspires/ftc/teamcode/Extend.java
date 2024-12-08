package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp()
public class Extend extends LinearOpMode {
    // ****** Extend variables ******
    // extend motor
    private DcMotorEx extend;
    // extend power
    private double extendPower;
    // PID coefficients for extend
    private final double kPExtend = 0.01; // adjust
    private final double kIExtend = 0; // adjust
    private final double kDExtend = 0; // adjust
    // Extend target position
    private static double extendTargetPos = 0; // adjust
    // Max extend length
    private double maxTargetExtend = 5000; // adjust
    // Extend sensitivity
    private double sensitivityExtend = 2.0; // adjust
    // Last error for extend PID
    private double lastErrorExtend = 0;
    // Integral error for extend PID
    private double errorSumExtend = 0;
    // Elapsed time for extend PID
    private ElapsedTime extendTimer;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // init extend
        initExtend();
        // telemetry
        extendTelemetry();
        
        waitForStart();
        // start timer
        extendTimer = new ElapsedTime();
        
        // TeleOp loop
        while (opModeIsActive()) {
            // use buttons to set target positions
            // extend buttons
            if (gamepad2.x) {
                extendTargetPos = 0; // adjust
            } else if (gamepad2.b) {
                extendTargetPos = 100; // adjust
            }

            // move extend with joysticks
            extendTargetPos = Math.round(extendTargetPos - sensitivityExtend * gamepad1.right_stick_y);
            
            // set motor power with PID
            extendPower = extendControl(extendTargetPos, extend.getCurrentPosition(), kPExtend, kIExtend, kDExtend);
            extend.setPower(extendPower);

            // telemetry
            extendTelemetry();
        }
    }
    
    // function to init extend
    public void initExtend() {
        extend = hardwareMap.get(DcMotorEx.class, "Extend"); // change device name
        extend.setDirection(DcMotor.Direction.REVERSE);
        extend.setPower(0);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    // function for pid control for extend
    public double extendControl(double target, double curPos, double kP1, double kI1, double kD1) {
        // error
        double error1 = target - curPos;
        // integral error
        errorSumExtend += error1 * extendTimer.seconds();
        // derivative
        double errorRate1 = (error1 - lastErrorExtend) / extendTimer.seconds();
        // reset last error
        lastErrorExtend = error1;
        // reset timer
        extendTimer.reset();
        // return power
        return kP1 * error1 + kI1 * errorSumExtend + kD1 * errorRate1;
    }
    
    public void extendTelemetry() {
        // telemetry
        telemetry.addData("Target For Extend", extendTargetPos);
        telemetry.addData("Current Position Of Extend", extend.getCurrentPosition());
        telemetry.addData("Sensitivity Of Extend", sensitivityExtend);
        telemetry.update();
    }
}
