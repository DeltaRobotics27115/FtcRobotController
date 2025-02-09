package com.deltarobotics27115.membercode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Sweep extends LinearOpMode {
    public Servo sweeper;

    @Override
    public void runOpMode() {
        sweeper = hardwareMap.get(Servo.class, "Sweeper");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        sweeper.setPosition(0);

        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0.2) {
                sweeper.setPosition(1);
                sleep(100);
                sweeper.setPosition(0);
            }
        }
    }
}