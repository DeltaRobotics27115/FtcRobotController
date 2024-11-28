package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;



public interface Subsystem {
    void init(HardwareMap hardwareMap);
    void start();
    void reset();
    void stop();
    State getState();

}
