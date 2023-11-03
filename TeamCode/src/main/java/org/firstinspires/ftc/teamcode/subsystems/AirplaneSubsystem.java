package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class AirplaneSubsystem extends SubsystemBase {
    private final Servo airplaneServer;

    public static double OPEN_POSITION = 1;
    public static double CLOSE_POSITION = 0;

    private boolean open = false;

    public AirplaneSubsystem(HardwareMap hardwareMap) {
        airplaneServer = hardwareMap.get(Servo.class, "airplaneServo");
    }

    public void open() {
        airplaneServer.setPosition(OPEN_POSITION);
        open = true;
    }

    public void close() {
        airplaneServer.setPosition(CLOSE_POSITION);
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
