package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeDefaultCommand extends CommandBase {
    DoubleSupplier trigger;
    IntakeSubsystem intake;

    public IntakeDefaultCommand(DoubleSupplier trigger, IntakeSubsystem intake) {
        this.trigger = trigger;
        this.intake = intake;

        addRequirements(intake);

    }

    public void execute() {
        intake.SetIntakePower(trigger.getAsDouble());
    }
}
