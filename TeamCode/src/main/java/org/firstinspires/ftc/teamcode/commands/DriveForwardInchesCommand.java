package org.firstinspires.ftc.teamcode.commands;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDriveSubsystem;

public class DriveForwardInchesCommand extends CommandBase {
    final SimpleMecanumDriveSubsystem drive;
    final double forward;

    //TODO:

    public static double MOTOR_TICS_PER_REVOLUTION = 8192;
    public static double WHEEL_DIAMETER_MM = 35;
    public static double DISTANCE_PER_REVOLUTION_MM = WHEEL_DIAMETER_MM * Math.PI;
    public static double TICS_PER_MM = MOTOR_TICS_PER_REVOLUTION / DISTANCE_PER_REVOLUTION_MM;
    public static double TICS_PER_INCHES = TICS_PER_MM * 25.4;

    int endTicks; // How far are we going?
    int curTicks = 0; // Where are we now
    int count = 0; // debug
    boolean forwardDir; // Forward or backwards?

    public DriveForwardInchesCommand(SimpleMecanumDriveSubsystem drive, double forward) {
        this.drive = drive;
        this.forward = forward;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        forwardDir = forward > 0;
        endTicks = drive.getTicks() + (int) (forward * TICS_PER_INCHES);
        double driveSpeed = .5;  // Remember, the stick is backwards!
        if (!forwardDir) {
            driveSpeed *= -1.0;
        }
        drive.drive(driveSpeed, 0, 0);
    }

    @Override
    public void execute() {
        curTicks = drive.getTicks();
        count++;
        telemetry.addData("count", count);
        telemetry.addData("endTicks", endTicks);
        telemetry.addData("curTicks", curTicks);
        telemetry.update();
    }

    @Override
    public boolean isFinished() {
        if (forwardDir) {
            if (curTicks >= endTicks) {
                return true;
            }
        } else {
            if (curTicks <= endTicks) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}