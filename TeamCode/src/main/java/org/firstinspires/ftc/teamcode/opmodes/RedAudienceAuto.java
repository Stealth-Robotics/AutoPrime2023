package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Trajectories.BlueAudience;
import org.firstinspires.ftc.teamcode.Trajectories.RedAudience;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commands.ResetElevatorCommand;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LeverSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PreloadHolder;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name = "RedAudienceAuto", preselectTeleOp = "RED | Tele-Op")
public class RedAudienceAuto extends StealthOpMode {

    SampleMecanumDrive mecanumDrive;
    ElevatorSubsystem elevator;
    CameraSubsystem camera;
    LeverSubsystem lever;
    ArmSubsystem arm;
    IntakeSubsystem intake;
    AirplaneSubsystem airplane;
    PreloadHolder preload;
    DriveSubsystem drive;

    public void initialize() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        drive = new DriveSubsystem(mecanumDrive, hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap, Alliance.RED);
        arm = new ArmSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        lever = new LeverSubsystem(hardwareMap);
        airplane = new AirplaneSubsystem(hardwareMap);
        preload = new PreloadHolder(hardwareMap);
        register(drive, elevator, camera, lever, intake, arm, airplane, preload);

        schedule(new ResetElevatorCommand(elevator));
    }

    public void whileWaitingToStart() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public Command getAutoCommand() {
        String ConeLocation = camera.getConePos();
        drive.setPoseEstimate(-38.5, -61, Math.toRadians(90));

        switch (ConeLocation) {
            case "left":
                return new SequentialCommandGroup(
                        new InstantCommand(() -> preload.close()),
                        new InstantCommand(() -> lever.close()),
                        new FollowTrajectory(mecanumDrive, RedAudience.scorepixelleft),
                        new InstantCommand(() -> preload.open()),
                        new FollowTrajectory(mecanumDrive, RedAudience.leftbackup)

                );
            case "right":
                return new SequentialCommandGroup(
                        new InstantCommand(() -> preload.close()),
                        new InstantCommand(() -> lever.close()),
                        new FollowTrajectory(mecanumDrive, RedAudience.scorepixelright),
                        new InstantCommand(() -> preload.open()),
                        new FollowTrajectory(mecanumDrive, RedAudience.rightforward)

                );

            default:
                return new SequentialCommandGroup(
                        new InstantCommand(() -> preload.close()),
                        new InstantCommand(() -> lever.close()),
                        new FollowTrajectory(mecanumDrive, RedAudience.scorepixelcenter),
                        new InstantCommand(() -> preload.open()),
                        new FollowTrajectory(mecanumDrive, RedAudience.centerbackup)

                );
        }
    }
}
