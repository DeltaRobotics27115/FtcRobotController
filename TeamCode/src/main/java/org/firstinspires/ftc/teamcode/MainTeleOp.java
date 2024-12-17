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
 * This is the main TeleOp OpMode for the robot.
 * It controls the robot's movements and actions based on gamepad input.
 */
@TeleOp
public class MainTeleOp extends LinearOpMode {

    private FieldCentricDriveControl fieldCentricDrive;
    private WristAndIntakeControl wristAndIntake;
    private ArmAndExtendControl armAndExtend;

    private static final double SLOW_MODE_TRIGGER_THRESHOLD = 0.1; // Define slow mode trigger threshold
    private static final double ARM_TARGET_POS_LOW = 0; // Define arm target positions
    private static final double ARM_TARGET_POS_HIGH = 2000;
    private static final double EXTEND_TARGET_POS_LOW = 0; // Define extend target positions
    private static final double EXTEND_TARGET_POS_HIGH = 450;

    /**
     * Runs the OpMode.
     * Initializes hardware, waits for start, and then continuously updates robot actions based on gamepad input.
     *
     * @throws InterruptedException If the OpMode is interrupted.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware components
        fieldCentricDrive = new FieldCentricDriveControl(hardwareMap);
        wristAndIntake = new WristAndIntakeControl(hardwareMap);
        armAndExtend = new ArmAndExtendControl(hardwareMap);

        waitForStart(); // Wait for the OpMode to start

        armAndExtend.startTimer(); // Start the timer for arm and extend control
        fieldCentricDrive.resetImu(); // Reset the IMU at the start

        // Main loop
        while (opModeIsActive()) {
            // Get gamepad input for drive control
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y; // Inverted y-axis for intuitive control
            double turn = gamepad1.right_stick_x;

            // Calculate slow mode amount based on right trigger
            double slowAmount = (x != 0 || y != 0 || turn != 0)
                    ? (gamepad1.right_trigger > SLOW_MODE_TRIGGER_THRESHOLD ? 0.3 : 0)
                    : 0;

            // Drive the robot using field-centric control
            DrivePower drivePower = fieldCentricDrive.driveFieldCentric(x, y, turn, slowAmount);

            // Update wrist and intake based on gamepad2 input
            WristAndIntakePower wristAndIntakePower = wristAndIntake.update(
                    gamepad2.left_trigger, gamepad2.right_trigger,
                    gamepad2.left_bumper, gamepad2.right_bumper);

            // Update arm and extend target positions based on gamepad2 buttons
            if (gamepad2.a) {
                armAndExtend.armTargetPos = ARM_TARGET_POS_LOW;
            } else if (gamepad2.y) {
                armAndExtend.armTargetPos = ARM_TARGET_POS_HIGH;
            }
            if (gamepad2.x) {
                armAndExtend.extendTargetPos = EXTEND_TARGET_POS_LOW;
            } else if (gamepad2.b) {
                armAndExtend.extendTargetPos = EXTEND_TARGET_POS_HIGH;
            }

            // Update arm and extend motor powers
            ArmAndExtendPower armAndExtendPower = armAndExtend.update(gamepad2.left_stick_y, gamepad2.right_stick_y);

            // Send telemetry data to the driver station
            telemetry.addData("Left Front Power", drivePower.leftFront);
            telemetry.addData("Right Front Power", drivePower.rightFront);
            telemetry.addData("Left Back Power", drivePower.leftBack);
            telemetry.addData("Right Back Power", drivePower.rightBack);
            telemetry.addData("Wrist Power", wristAndIntakePower.wristPower);
            telemetry.addData("Intake Power", wristAndIntakePower.intakePower);
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