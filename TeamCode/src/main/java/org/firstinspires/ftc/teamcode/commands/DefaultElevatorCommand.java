package org.firstinspires.ftc.teamcode.commands;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Keep moving the elevator to the target position we set elsewhere.
 * This command never ends, and just keeps asking the elevator subsystem to update its position.
 */

@Config


public class DefaultElevatorCommand extends CommandBase {
    final ElevatorSubsystem elevator;
    final DoubleSupplier trigger;
    public static double manualESpeed = 0.01;

    public DefaultElevatorCommand(ElevatorSubsystem elevator,
                                  DoubleSupplier trigger
    ) {
        this.elevator = elevator;
        this.trigger = trigger;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        //elevator.goToPosition();
        //double newPos = 0;
        //double currentPos = elevator.getTargetLocation();
        double power = trigger.getAsDouble();
        if(elevator.getState() == ElevatorSubsystem.ElevatorState.ABOVE_MAXIMUM){
            power = MathUtils.clamp(power, -1, 0);
        }
        else if(elevator.getState() == ElevatorSubsystem.ElevatorState.BELOW_MINIMUM){
            power = MathUtils.clamp(power, 0, 1);
        }
        else {
            power = MathUtils.clamp(power, -1, 1);
        }
        elevator.setPower(power);



        //elevator.setTargetLocation(elevator.getTargetLocation());

    }

}