package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.output.DrivePower;

/**
 * This class provides field-centric drive control for a robot with four mecanum wheels.
 */
public class FieldCentricDriveControl {

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final BHI260IMU imu;

    /**
     * Constructor for FieldCentricDriveControl.
     * Initializes the motors and IMU.
     *
     * @param hardwareMap The hardware map to access the motors and IMU.
     */
    public FieldCentricDriveControl(HardwareMap hardwareMap) {
        // Initialize motors and reverse directions as needed
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
    }

    /**
     * Calculates and sets the motor powers for field-centric drive.
     *
     * @param x          The desired movement in the x-direction (strafe).
     * @param y          The desired movement in the y-direction (forward/backward).
     * @param turn       The desired rotation.
     * @param slowAmount The amount to reduce the overall power (slow mode).
     * @return A DrivePower object containing the calculated motor powers.
     */
    public DrivePower driveFieldCentric(double x, double y, double turn, double slowAmount) {
        // Get current robot heading from IMU
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate input vector by robot heading to achieve field-centric control
        double rotatedX = x * Math.cos(heading) + y * Math.sin(heading);
        double rotatedY = -x * Math.sin(heading) + y * Math.cos(heading);

        // Calculate individual motor powers
        double leftFront = rotatedY + rotatedX + turn;
        double rightFront = rotatedY - rotatedX - turn;
        double leftBack = rotatedY - rotatedX + turn;
        double rightBack = rotatedY + rotatedX - turn;

        // Normalize motor powers to prevent exceeding maximum power
        double maxPower = Math.max(Math.abs(leftFront), Math.max(Math.abs(rightFront),
                Math.max(Math.abs(leftBack), Math.abs(rightBack))));
        if (maxPower > 1) {
            leftFront /= maxPower;
            rightFront /= maxPower;
            leftBack /= maxPower;
            rightBack /= maxPower;
        }
        // Apply slow mode correctly
        leftFront = applySlowMode(leftFront, slowAmount);
        rightFront = applySlowMode(rightFront, slowAmount);
        leftBack = applySlowMode(leftBack, slowAmount);
        rightBack = applySlowMode(rightBack, slowAmount);

        // Apply slow mode and set motor powers
        frontLeft.setPower(leftFront);
        frontRight.setPower(rightFront);
        backLeft.setPower(leftBack);
        backRight.setPower(rightBack);

        // Return calculated motor powers
        return new DrivePower(leftFront - slowAmount, rightFront - slowAmount, leftBack - slowAmount, rightBack - slowAmount);
    }
    private double applySlowMode(double power, double slowAmount) {
        if (power > 0) {
            return Math.max(0, power - slowAmount); // Don't go below 0
        } else if (power < 0) {
            return Math.min(0, power + slowAmount); // Don't go above 0
        } else {
            return 0; // Already 0, stay at 0
        }
    }
    /**
     * Resets the IMU's yaw angle to zero.
     */
    public void resetImu() {
        imu.resetYaw();
    }
}