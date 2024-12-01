package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class WristAndIntake extends LinearOpMode {
    public Servo wrist;
    public CRServo intake;



    @Override
    public void runOpMode() {
        wrist = hardwareMap.get(Servo.class, "Wrist");
        intake = hardwareMap.get(CRServo.class, "Intake");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //wait for game to start
        waitForStart();

        wrist.setPosition(0.5);

        //run until end
        while (opModeIsActive()) {
            //wrist left and right
            if (gamepad2.dpad_left) {
                wrist.setPosition(1);
            } else if (gamepad2.dpad_right) {
                wrist.setPosition(0);
            } else {
                wrist.setPosition(0.5);
            }

            if (gamepad2.left_bumper) {
                intake.setPower(1);
            } else if (gamepad2.right_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
        }
    }
}
