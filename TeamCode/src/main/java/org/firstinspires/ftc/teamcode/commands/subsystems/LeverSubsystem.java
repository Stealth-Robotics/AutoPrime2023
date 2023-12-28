package org.firstinspires.ftc.teamcode.commands.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class LeverSubsystem extends SubsystemBase {

    private final Servo leverServo;

    public static double OPEN_POSITION = 1;
    public static double CLOSE_POSITION = 0.5;

    private boolean open = false;

    public LeverSubsystem(HardwareMap hardwareMap) {
        leverServo = hardwareMap.get(Servo.class, "leverServo");
    }

    public void open() {
        leverServo.setPosition(OPEN_POSITION);
        open = true;
    }

    public void close() {
        leverServo.setPosition(CLOSE_POSITION);
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
        telemetry.addData("Gripper position", leverServo.getPosition());
    }
}
