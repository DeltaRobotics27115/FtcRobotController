package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.ArmAndExtendControl;
import org.firstinspires.ftc.teamcode.hardware.FieldCentricDriveControl;
import org.firstinspires.ftc.teamcode.hardware.WristAndIntakeControl;
import org.firstinspires.ftc.teamcode.output.ArmAndExtendPower;
import org.firstinspires.ftc.teamcode.output.DrivePower;
import org.firstinspires.ftc.teamcode.output.WristAndIntakePower;

/**
 *
 */
@TeleOp
public class MainTeleOp extends LinearOpMode {
    public FieldCentricDriveControl fieldCentricDrive;
    public WristAndIntakeControl wristAndIntake;
    public ArmAndExtendControl armAndExtend;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize FieldCentricDrive
        fieldCentricDrive = new FieldCentricDriveControl(hardwareMap);
        // Initialize WristAndIntake
        wristAndIntake = new WristAndIntakeControl(hardwareMap);
        // Initialize ArmAndExtendControl
        armAndExtend = new ArmAndExtendControl(hardwareMap);
        waitForStart();
        armAndExtend.startArmTimer();
        fieldCentricDrive.resetImu();//Reset IMU at start
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double slowAmount = (x != 0 || y != 0 || turn != 0) ?
                    (gamepad1.right_trigger > 0.1 ? 0.3 : 0) : 0;
            telemetry.addData(
                    "X", x
            );
            telemetry.addData(
                    "Y", y
            );
            telemetry.addData(
                    "Turn", turn
            );
            telemetry.addData(
                    "SlowAmount", slowAmount
            );
            telemetry.addData(
                    "Right_trigger", gamepad1.right_trigger
            );


            // Use the FieldCentricDrive object to drive
            DrivePower drivePower = fieldCentricDrive.driveFieldCentric(x, y, turn, slowAmount);
            telemetry.addData("Left Front Power", drivePower.leftFront);
            telemetry.addData("Right Front Power", drivePower.rightFront);
            telemetry.addData("Left Back Power", drivePower.leftBack);
            telemetry.addData("Right Back Power", drivePower.rightBack);

            // Use the WristAndIntake object to update wrist and intake
            WristAndIntakePower wristAndIntakePower = wristAndIntake.update(gamepad2.left_trigger, gamepad2.right_trigger,
                    gamepad2.left_bumper, gamepad2.right_bumper);
            telemetry.addData("Wrist Power", wristAndIntakePower.wristPower);
            telemetry.addData("Intake Power", wristAndIntakePower.intakePower);
            // Use the ArmAndExtendControl object to update arm and extend
            // use buttons to set target positions
            // arm buttons
            if (gamepad2.a) {
                armAndExtend.armTargetPos = 0; // adjust
            } else if (gamepad2.y) {
                armAndExtend.armTargetPos = 2000; // adjust
            }
            // extend buttons
            if (gamepad2.x) {
                armAndExtend.extendTargetPos = 0; // adjust
            } else if (gamepad2.b) {
                armAndExtend.extendTargetPos = 450; // adjust
            }
            ArmAndExtendPower armAndExtendPower = armAndExtend.update(gamepad2.left_stick_y, gamepad2.right_stick_y);
            telemetry.addData("Target For Arm", armAndExtendPower.armTargetPos);
            telemetry.addData("Current Position Of Arm", armAndExtendPower.armPosition);
            telemetry.addData("Sensitivity Of Arm", armAndExtendPower.sensitivityArmInit * armAndExtendPower.scaleFactorArm);
            telemetry.addData("Target For Extend", armAndExtendPower.extendTargetPos);
            telemetry.addData("Current Position Of Extend", armAndExtendPower.extendPosition);
            telemetry.addData("Sensitivity Of Extend", armAndExtendPower.sensitivityExtendInit);
            telemetry.update();
        }


    }
}
