package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;

public class IntakeDefaultCommand extends CommandBase {
    BooleanSupplier forward;
    BooleanSupplier reverse;
    IntakeSubsystem intake;

    public IntakeDefaultCommand(IntakeSubsystem intake, BooleanSupplier forward, BooleanSupplier reverse) {
        this.forward = forward;
        this.reverse = reverse;
        this.intake = intake;

        addRequirements(intake);

    }

    public void execute() {
        if(forward.getAsBoolean()){
            intake.SetIntakePower(1);
        }
        else if(reverse.getAsBoolean()){
            intake.SetIntakePower(-1);
        }
        else{
            intake.SetIntakePower(0);
        }
    }
}
