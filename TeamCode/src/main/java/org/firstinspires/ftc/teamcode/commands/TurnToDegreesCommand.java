package org.firstinspires.ftc.teamcode.commands;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.subsystems.SimpleMecanumDriveSubsystem;

@Config
public class TurnToDegreesCommand extends CommandBase {
    final SimpleMecanumDriveSubsystem drive;
    double toRadians;

    //TODO: tune pid
    public static double kp = 0.5;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0.1;

    PIDFController pid = new PIDFController(kp, ki, kd, kf);

    public TurnToDegreesCommand(SimpleMecanumDriveSubsystem drive, double degrees) {
        this.drive = drive;
        toRadians = Math.toRadians(degrees);
        addRequirements(drive);
        pid.setTolerance(5);
    }

    @Override
    public void initialize() {
        pid.setSetPoint(drive.getHeading() + toRadians);

    }

    @Override
    public void execute() {
        double currentRadians = drive.getHeading();
        double delta = AngleUnit.normalizeRadians(toRadians - currentRadians);

        pid.setPIDF(kp, ki, kd, kf);
        double power = pid.calculate(-delta);
        double maxSpeed = 0.5;
        power = MathUtils.clamp(power, -maxSpeed, maxSpeed);

        telemetry.addData("toRadians", toRadians);
        telemetry.addData("currentRadians", currentRadians);
        telemetry.addData("delta", delta);
        telemetry.update();

        drive.drive(0, 0, power);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}