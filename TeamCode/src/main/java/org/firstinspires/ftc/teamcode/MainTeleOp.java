package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.ArmAndExtendControl;
import org.firstinspires.ftc.teamcode.hardware.FieldCentricDriveControl;
import org.firstinspires.ftc.teamcode.hardware.WristAndIntakeControl;
import org.firstinspires.ftc.teamcode.output.ArmAndExtendPower;
import org.firstinspires.ftc.teamcode.output.DrivePower;
import org.firstinspires.ftc.teamcode.output.WristAndIntakePower;
import org.firstinspires.ftc.teamcode.util.ArmAndExtendState;

/**
 * This is the main TeleOp OpMode for the robot.
 * It controls the robot's movements and actions based on gamepad input.
 */
@Config
@TeleOp

public class MainTeleOp extends LinearOpMode {

    public static double kP=0.01;
    public static double kI=0.00;
    public static double kD=0.0;
    public static double kPextend=0.01;
    public static double kIextend=0.00;
    public static double kDextend=0.0;
    public static double extendCatchPosition=2050;
    public static double armMaxPosition=3000;
    public static double slowMode=0.5;
    public  static  double armShootPosition=1700;
    public  static  double extendShootPosition=2050;

    private FieldCentricDriveControl fieldCentricDrive;
    private WristAndIntakeControl wristAndIntake;
    private ArmAndExtendControl armAndExtend;
    public Servo sweeper;
    public static double perDistcnce=100;
    public static double wristIncrement=0.02;
    private final double SLOW_MODE_TRIGGER_THRESHOLD = 0.3;
    private ArmAndExtendPower armAndExtendPower;
    private ArmAndExtendState armAndExtendState=ArmAndExtendState.TO_UPDATE;


     // Define slow mode trigger threshold


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
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sweeper=hardwareMap.get(Servo.class,"Sweeper");

        waitForStart(); // Wait for the OpMode to start

        armAndExtend.initPosition();
        fieldCentricDrive.resetImu();
        wristAndIntake.setWristPosition(0.5);
        sweeper.setPosition(0);

        // Main loop
        while (opModeIsActive()) {
            armAndExtend.setExtendMaxPosition(extendCatchPosition);
            armAndExtend.setArmMaxPosition(armMaxPosition);
            armAndExtend.setArmPID(kP,kI,kD);
            armAndExtend.setExtendPID(kPextend,kIextend,kDextend);
            wristAndIntake.setWristIncrement(wristIncrement);

            // Get gamepad input for drive control
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y; // Inverted y-axis for intuitive control
            double turn = gamepad1.right_stick_x;

            // Calculate slow mode amount based on right trigger
            double slowAmount = (x != 0 || y != 0 || turn != 0)
                    ? (gamepad1.right_trigger > SLOW_MODE_TRIGGER_THRESHOLD ? slowMode : 0)
                    : 0;

            // Drive the robot using field-centric control
            DrivePower drivePower = fieldCentricDrive.driveFieldCentric(x, y, turn, slowAmount);

            // Update wrist and intake based on gamepad2 input
            WristAndIntakePower wristAndIntakePower = wristAndIntake.update(
                    gamepad2.left_trigger, gamepad2.right_trigger,
                    gamepad2.left_bumper, gamepad2.right_bumper);

            // Update arm and extend target positions based on gamepad2 buttons
            if (gamepad2.a) {
                armAndExtendState=ArmAndExtendState.TO_CATCH;
            } else if (gamepad2.b) {
                armAndExtendState=ArmAndExtendState.TO_MOVE;
            }else if(gamepad2.x){
              armAndExtendState=ArmAndExtendState.TO_SHOOT;
            }
            else if(gamepad2.left_stick_y!=0 || gamepad2.right_stick_y!=0){
                armAndExtendState=ArmAndExtendState.TO_UPDATE;
            }
            switch (armAndExtendState){
                case TO_CATCH:
                    armAndExtendPower=armAndExtend.toCatchPosition();
                    break;
                case TO_MOVE:
                    armAndExtendPower=armAndExtend.toMovePosition();
                    break;
                case TO_SHOOT:
                    armAndExtendPower=armAndExtend.toShootPosition(armShootPosition,extendShootPosition);
                    break;
                default:
                    armAndExtendPower=armAndExtend.update(gamepad2.left_stick_y, gamepad2.right_stick_y,perDistcnce);
                    break;
            }

            //sweeper
            if(gamepad1.left_trigger>0.2){
                sweeper.setPosition(1);
                sleep(100);
                sweeper.setPosition(0);
            }


            // Send telemetry data to the driver station
            telemetry.addData("Left Front Power", drivePower.leftFront);
            telemetry.addData("Right Front Power", drivePower.rightFront);
            telemetry.addData("Left Back Power", drivePower.leftBack);
            telemetry.addData("Right Back Power", drivePower.rightBack);
            telemetry.addData("Wrist Position", wristAndIntake.getWristPosition());
            telemetry.addData("Target For Arm", armAndExtendPower.armTargetPos);
            telemetry.addData("Current Position Of Arm", armAndExtend.getArmPosition());
            telemetry.addData("Target For Extend", armAndExtendPower.extendTargetPos);
            telemetry.addData("Current Position Of Extend", armAndExtend.getExtendPosition());

            telemetry.update();
        }
    }
}