package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class MoveElevatorPercentage extends CommandBase {
    double distance;
    final ElevatorSubsystem elevator;

    public MoveElevatorPercentage(ElevatorSubsystem elevator, double distance) {
        this.elevator = elevator;
        this.distance = distance;
        addRequirements(elevator);
    }
}
