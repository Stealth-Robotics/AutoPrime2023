package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PreloadHolder extends SubsystemBase {
    private final Servo holderServo;

    public static double OPEN_POSITION = 0.2;
    public static double CLOSE_POSITION = 0.1;

    public boolean open = false;

    public PreloadHolder(HardwareMap hardwareMap) {
        holderServo = hardwareMap.get(Servo.class, "holderServo");
        close();
    }

    public void open() {
        holderServo.setPosition(OPEN_POSITION);
        open = true;
    }

    public void close() {
        holderServo.setPosition(CLOSE_POSITION);
        open = false;
    }

    public void toggle() {
        if (open) {
            close();
        }else{
            open();
        }
    }

    public void periodic() {
    }
}
