package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private final double kP;
    private final double kI;
    private final double kD;
    private double lastError, errorSum = 0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double target, double currentPosition, ElapsedTime timer) {
        double error = target - currentPosition;
        errorSum += error * timer.seconds();
        double errorRate = (error - lastError) / timer.seconds();

        double output = kP * error + kI * errorSum + kD * errorRate;

        lastError = error;
        timer.reset();

        return output;
    }
}
