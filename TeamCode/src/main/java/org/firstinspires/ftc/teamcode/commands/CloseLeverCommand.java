package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystems.LeverSubsystem;

public class CloseLeverCommand extends SequentialCommandGroup {
    public CloseLeverCommand(LeverSubsystem lever){
        addCommands(
                new InstantCommand(() -> lever.close()),
                new WaitCommand(1000)
        );
    }
}