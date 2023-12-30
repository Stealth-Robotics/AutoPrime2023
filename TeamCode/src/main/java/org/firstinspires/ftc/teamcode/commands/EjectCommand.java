
package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class EjectCommand extends SequentialCommandGroup{
    public EjectCommand(IntakeSubsystem intake){
        addCommands(new InstantCommand(()-> intake.SetIntakePower(1)), new WaitCommand(1500), new InstantCommand(()-> intake.SetIntakePower(0)));
    }
}