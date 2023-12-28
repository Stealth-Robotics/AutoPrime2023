package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commands.subsystems.ClimberSubsystem;

import java.util.function.BooleanSupplier;

public class DefaultClimberCommand extends CommandBase {
    private final BooleanSupplier leftBumper, rightBumper;
    private final ClimberSubsystem climber;

    public DefaultClimberCommand(ClimberSubsystem climber, BooleanSupplier rightBumper, BooleanSupplier leftBumper){
        this.climber = climber;
        this.rightBumper = rightBumper;
        this.leftBumper = leftBumper;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        if(leftBumper.getAsBoolean()){

            //TODO: PROBABLY WILL WANT TO UP THIS
            climber.SetPower(1);
        }
        else if(rightBumper.getAsBoolean()){
            //TODO: PROBABLY WILL WANT TO UP THIS

            climber.SetPower(-1);
        }
        else{
            climber.SetPower(0);
        }
    }
}
