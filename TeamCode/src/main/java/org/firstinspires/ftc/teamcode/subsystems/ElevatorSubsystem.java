package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorSubsystem extends SubsystemBase {
    private DcMotorEx elevatorMotor;


    public ElevatorSubsystem(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);//TODO: check if right direction
    }

    public void SetElevatorPower(double power) {
        elevatorMotor.setPower(power);
    }
}