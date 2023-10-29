package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem extends SubsystemBase {

    private final Servo armServo;

    public static double UP_POSITION = 0;
    public static double DOWN_POSITION = 0;

    private boolean up = false;

    public ArmSubsystem(HardwareMap hardwareMap) {
        armServo = hardwareMap.get(Servo.class, "armServo");
    }

    public void up() {
        armServo.setPosition(UP_POSITION);
        up = true;
    }

    public void down() {
        armServo.setPosition(DOWN_POSITION);
        up = false;
    }

    public void toggle() {
        if (up) {
            down();
        }else{
            up();
        }
    }

    public void periodic() {
        telemetry.addData("Arm position", armServo.getPosition());
    }
}
