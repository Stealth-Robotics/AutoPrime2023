package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.Teleop;

@Config
public class ArmSubsystem extends SubsystemBase {

    private final Servo armServo;

    public static double SCORE_POSITION = 0;
    public static double INTAKE_POSITION = 1;

    private boolean up = false;

    public ArmSubsystem(HardwareMap hardwareMap) {
        armServo = hardwareMap.get(Servo.class, "armServo");
    }

    public void scorePosition() {
        armServo.setPosition(SCORE_POSITION);
        up = true;
    }

    public void intakePosition() {
        armServo.setPosition(INTAKE_POSITION);
        up = false;
    }

    public void GoToPosition(double position) {
        armServo.setPosition(position);
    }

    public void toggle() {
        if (up) {
            intakePosition();
        }else{
            scorePosition();
        }
    }

    public void periodic() {
        telemetry.addData("Arm position", armServo.getPosition());
    }
}
