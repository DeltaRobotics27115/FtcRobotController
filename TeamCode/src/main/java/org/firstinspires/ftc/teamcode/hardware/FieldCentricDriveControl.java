package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.output.DrivePower;

public class FieldCentricDriveControl {
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final BHI260IMU imu;

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

    public DrivePower driveFieldCentric(double x, double y, double turn, double slowAmount) {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate input vector by IMU heading
        double rotatedX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotatedY = x * Math.sin(heading) + y * Math.cos(heading);

        double leftFront = rotatedY + rotatedX + turn;
        double rightFront = rotatedY - rotatedX - turn;
        double leftBack = rotatedY - rotatedX + turn;
        double rightBack = rotatedY + rotatedX - turn;

        // Normalize motor powers
        double maxPower = Math.max(Math.abs(leftFront), Math.max(Math.abs(rightFront),
                Math.max(Math.abs(leftBack), Math.abs(rightBack))));
        if (maxPower > 1) {
            leftFront /= maxPower;
            rightFront /= maxPower;
            leftBack /= maxPower;
            rightBack /= maxPower;
        }
        frontLeft.setPower(leftFront - slowAmount);
        frontRight.setPower(rightFront - slowAmount);
        backLeft.setPower(leftBack - slowAmount);
        backRight.setPower(rightBack - slowAmount);
        return new DrivePower(leftFront - slowAmount, rightFront - slowAmount, leftBack - slowAmount, rightBack - slowAmount);
    }

    public void resetImu() {
        imu.resetYaw();
    }
}
