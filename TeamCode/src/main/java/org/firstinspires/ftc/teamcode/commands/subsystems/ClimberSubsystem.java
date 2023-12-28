package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClimberSubsystem extends SubsystemBase {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
    public ClimberSubsystem(HardwareMap hardwareMap){
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftArmMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightArmMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void SetPower(double power){
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
}
