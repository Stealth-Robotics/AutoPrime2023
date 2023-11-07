package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefaultElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ResetElevatorCommand;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LeverSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.DefaultMecanumDriveCommand;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public abstract class Teleop extends StealthOpMode {

    // Subsystems
    SimpleMecanumDriveSubsystem drive;
    IntakeSubsystem intake;

    ElevatorSubsystem elevator;

    ArmSubsystem arm;

    LeverSubsystem lever;

    CameraSubsystem camera;

    AirplaneSubsystem airplane;

    // Game controllers
    GamepadEx driveGamepad;
    GamepadEx mechGamepad;


    @Override
    public void initialize() {
        // Setup and register all of your subsystems here
        drive = new SimpleMecanumDriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        lever = new LeverSubsystem(hardwareMap);
        arm  = new ArmSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap, Alliance.RED);
        airplane = new AirplaneSubsystem(hardwareMap);

        register(drive);

        driveGamepad = new GamepadEx(gamepad1);
        mechGamepad = new GamepadEx(gamepad2);

        // Automatically reset the elevator all the way down when we init
//        schedule(new ResetElevatorCommand(elevator));

        // A subsystem's default command runs all the time. Great for drivetrains and such.
        drive.setDefaultCommand(
                new DefaultMecanumDriveCommand(
                        drive,
                        () -> driveGamepad.gamepad.left_stick_y,
                        () -> driveGamepad.gamepad.left_stick_x,
                        () -> driveGamepad.gamepad.right_stick_x
                )
        );

        elevator.setDefaultCommand(new DefaultElevatorCommand(elevator,
                        () -> (mechGamepad.gamepad.left_trigger - mechGamepad.gamepad.right_trigger)

                )
        );

        intake.setDefaultCommand(
                new IntakeDefaultCommand(
                        intake,
                        () -> driveGamepad.gamepad.right_bumper,
                        () -> driveGamepad.gamepad.left_bumper
                )
        );

        // Setup all of your controllers' buttons and triggers here
        mechGamepad.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new ResetElevatorCommand(elevator));
        driveGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> drive.resetHeading()));
        driveGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(()-> drive.toggleFieldCentric()));
        driveGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> drive.toggleSlowMode()));

        mechGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> lever.toggle()));
        mechGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> arm.toggle()));
        mechGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> airplane.toggle()));
    }

    /**
     * Ideally your red vs. blue opmodes are nothing more than this. Keep code shared between
     * them, and take different actions based on the alliance color.
     *
     * @see org.stealthrobotics.library.Alliance
     */

    @SuppressWarnings("unused")
    @TeleOp(name = "RED | Tele-Op", group = "Red")
    public static class RedTeleop extends Teleop {
    }

    @SuppressWarnings("unused")
    @TeleOp(name = "BLUE | Tele-Op", group = "Blue")
    public static class BlueTeleop extends Teleop {
    }

    @Override
    public double getFinalHeading() {
        return drive.getHeading();
    }

}
