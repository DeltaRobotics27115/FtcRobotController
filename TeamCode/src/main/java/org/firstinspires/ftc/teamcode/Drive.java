package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Drive extends LinearOpMode {
    public Blinker control_Hub;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //wait for game to start
        waitForStart();
        double slowammount = 0;
        //run until end
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);
            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.max(Math.abs(sin),
                    Math.abs(cos));
            double leftFront = power * cos/max + turn;
            double rightFront = power * sin/max - turn;
            double leftBack = power * sin/max + turn;
            double rightBack = power * cos/max - turn;
            if ((power + Math.abs(turn)) > 1) {
                leftFront /= power + turn;
                rightFront /= power + turn;
                leftBack /= power + turn;
                rightBack /= power + turn;
            }
            if (gamepad1.right_trigger > 0.1) {
                slowammount = 0.3;
            } else {
                slowammount = 0;
            }
            frontLeft.setPower(leftFront - slowammount);
            frontRight.setPower(rightFront - slowammount);
            backLeft.setPower(leftBack - slowammount);
            backRight.setPower(rightBack - slowammount);
        }
    }
}