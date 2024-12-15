package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp()
public class PIDArm extends LinearOpMode {
    // ****** Arm variables ******
    // Arm motor
    private DcMotorEx arm;
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


    // ****** Extend variables ******
    // extend motor
    private DcMotorEx extend;
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
        // init arm and extend
        initArm();
        initExtend();
        // telemetry
        armAndExtendTelemetry();


        waitForStart();
        // start timers
        armTimer = new ElapsedTime();
        extendTimer = new ElapsedTime();


        // TeleOp loop
        while (opModeIsActive()) {
            // use buttons to set target positions
            // arm buttons
            if (gamepad2.a) {
                armTargetPos = -120; // adjust
            } else if (gamepad2.y) {
                armTargetPos = 2000; // adjust
            }
            // extend buttons
            if (gamepad2.x) {
                extendTargetPos = 0; // adjust
            } else if (gamepad2.b) {
                extendTargetPos = 100; // adjust
            }


            // move arm and extend with joysticks
            extendTargetPos = Math.round(extendTargetPos - sensitivityExtend * gamepad2.right_stick_y);
		if (extendTargetPos > 450) extendTargetPos = 450;
            scaleFactorArm = 1 - (Math.abs(extendTargetPos) / maxTargetExtend);
            armTargetPos = Math.round(armTargetPos - sensitivityArmInit * scaleFactorArm * gamepad2.left_stick_y);]


            // set motor power with PID
            double power1 = armControl(armTargetPos, arm.getCurrentPosition(), kPArm, kIArm, kDArm);
            double power2 = extendControl(extendTargetPos, extend.getCurrentPosition(), kPExtend, kIExtend, kDExtend);
            arm.setPower(power1);
            extend.setPower(power2);


            // telemetry
            armAndExtendTelemetry();
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


    // function to init extend
    public void initExtend() {
        extend = hardwareMap.get(DcMotorEx.class, "Extend"); // change device name
        extend.setDirection(DcMotor.Direction.REVERSE);
        extend.setPower(0);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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


    // function to display motor status
    public void armAndExtendTelemetry() {
        // telemetry
        telemetry.addData("Target For Arm", armTargetPos);
        telemetry.addData("Current Position Of Arm", arm.getCurrentPosition());
        telemetry.addData("Sensitivity Of Arm", sensitivityArmInit * scaleFactorArm);
        telemetry.addData("Target For Extend", extendTargetPos);
        telemetry.addData("Current Position Of Extend", extend.getCurrentPosition());
        telemetry.addData("Sensitivity Of Extend", sensitivityExtend);
        telemetry.update();
    }
}
