package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveForwardInchesCommand;
import org.firstinspires.ftc.teamcode.commands.TurnToDegreesCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.AirplaneSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.LeverSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.SimpleMecanumDriveSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name = "RedAudienceAuto", preselectTeleOp = "RED | Tele-Op")
public class RedAudienceAuto extends StealthOpMode {

    ElevatorSubsystem elevator;

    SimpleMecanumDriveSubsystem drive;

    CameraSubsystem camera;
    LeverSubsystem lever;
    ArmSubsystem arm;
    IntakeSubsystem intake;
    AirplaneSubsystem airplane;

    public void initialize() {
        drive = new SimpleMecanumDriveSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap, Alliance.RED);
        arm = new ArmSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        lever = new LeverSubsystem(hardwareMap);
        airplane = new AirplaneSubsystem(hardwareMap);
        register(drive, elevator, camera, lever, intake, arm, airplane);

        // tell the camera, we are starting on a specific side of the field
    }

    public void whileWaitingToStart() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public Command getAutoCommand() {
        String ConeLocation = camera.getConePos();
        //guesses center for now because the angle and camera don't work rn

        switch (ConeLocation){
            case "left":
                return new SequentialCommandGroup(
                        new DriveForwardInchesCommand(drive,27),
                        new DriveForwardInchesCommand(drive,-2),
                        new TurnToDegreesCommand(drive, -98),
                        new WaitCommand(500),
                        new InstantCommand(()-> intake.SetIntakePower(0.65)),
                        new WaitCommand(1800),
                        new InstantCommand(()-> intake.SetIntakePower(0))

                );
            case "right":
                return new SequentialCommandGroup(
                        new DriveForwardInchesCommand(drive,27),
                        new DriveForwardInchesCommand(drive,-2),
                        new TurnToDegreesCommand(drive, 98),
                        new WaitCommand(500),
                        new InstantCommand(()-> intake.SetIntakePower(0.65)),
                        new WaitCommand(1800),
                        new InstantCommand(()-> intake.SetIntakePower(0))

                );

            default:
                return new SequentialCommandGroup(
                        new DriveForwardInchesCommand(drive,29),
                        new DriveForwardInchesCommand(drive,-6),
                        new DriveForwardInchesCommand(drive,4),
                        new WaitCommand(500),
                        new InstantCommand(()-> intake.SetIntakePower(0.65)),
                        new WaitCommand(1800),
                        new InstantCommand(()-> intake.SetIntakePower(0))
                );

        }
    }
}
