package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.LeverSubsystem;

public class OpenLeverCommand extends SequentialCommandGroup {
    public OpenLeverCommand(LeverSubsystem lever){
        addCommands(
                new InstantCommand(() -> lever.open()),
                new WaitCommand(1000)
        );
    }
}