package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Trajectories.RedRightTrajectories;
import org.firstinspires.ftc.teamcode.commands.CloseLeverCommand;
import org.firstinspires.ftc.teamcode.commands.DefaultElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.DriveForwardInchesCommand;
import org.firstinspires.ftc.teamcode.commands.EjectCommand;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commands.MoveElevatorPercentage;
import org.firstinspires.ftc.teamcode.commands.OpenLeverCommand;
import org.firstinspires.ftc.teamcode.commands.ResetElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.StrafeForInches;
import org.firstinspires.ftc.teamcode.commands.TurnToDegreesCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LeverSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PreloadHolder;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDriveSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name = "RedRightSideStart", preselectTeleOp = "RED | Tele-Op")
public class RedRStartAuto extends StealthOpMode {

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
    @Override
    public void whileWaitingToStart() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public Command getAutoCommand() {
        String ConeLocation = camera.getConePos();
        drive.setPoseEstimate(15, -60, Math.toRadians(90));

        switch (ConeLocation) {
            case "left":
                return new SequentialCommandGroup(
                        new InstantCommand(() -> preload.close()),
                        new InstantCommand(()-> lever.close()),
                        new FollowTrajectory(mecanumDrive, RedRightTrajectories.scorepixelleft),
                        new InstantCommand(() -> preload.open()),
                        new WaitCommand(200),
                        new FollowTrajectory(mecanumDrive, RedRightTrajectories.trajectory10),
                        new ParallelCommandGroup(
                                new MoveElevatorPercentage(elevator, 0.4),
                                new InstantCommand(()-> arm.intakePosition())
                        ),
                        new WaitCommand(1500),
                        new FollowTrajectory(mecanumDrive, RedRightTrajectories.trajectory11),
                        new OpenLeverCommand(lever),
                        new MoveElevatorPercentage(elevator, 0.42),
                        new FollowTrajectory(mecanumDrive, RedRightTrajectories.trajectory12),
                        new FollowTrajectory(mecanumDrive, RedRightTrajectories.leftpark),
                        new InstantCommand(()-> arm.scorePosition()),
                        new WaitCommand(1000),
                        new ResetElevatorCommand(elevator)

                );
            case "right":
                return new SequentialCommandGroup(
                        new InstantCommand(() -> preload.close()),
                        new InstantCommand(()-> lever.close()),
                        new FollowTrajectory(mecanumDrive, RedRightTrajectories.scorepixelright),
                        new InstantCommand(() -> preload.open()),
                        new WaitCommand(200),
                        new FollowTrajectory(mecanumDrive, RedRightTrajectories.trajectory4),
                        new ParallelCommandGroup(
                                new MoveElevatorPercentage(elevator, 0.4),
                                new InstantCommand(()-> arm.intakePosition())
                        ),
                        new WaitCommand(1500),
                        new FollowTrajectory(mecanumDrive, RedRightTrajectories.trajectory8),
                        new OpenLeverCommand(lever),
                        new MoveElevatorPercentage(elevator, 0.42),
                        new FollowTrajectory(mecanumDrive, RedRightTrajectories.trajectory9),
                        new FollowTrajectory(mecanumDrive, RedRightTrajectories.rightpark),
                        new InstantCommand(()-> arm.scorePosition()),
                        new WaitCommand(1000),
                        new ResetElevatorCommand(elevator)
                );

            default:
                return new SequentialCommandGroup(
                        new InstantCommand(()-> preload.close()),
                        new InstantCommand(()-> lever.close()),
                        new FollowTrajectory(mecanumDrive, RedRightTrajectories.scorepixelcenter),
                        new InstantCommand(()-> preload.open()),
                        new FollowTrajectory(mecanumDrive, RedRightTrajectories.trajectory2),
                        new FollowTrajectory(mecanumDrive, RedRightTrajectories.trajectory3),
                        new ParallelCommandGroup(
                                new MoveElevatorPercentage(elevator, 0.4),
                                new InstantCommand(()-> arm.intakePosition())
                        ),
                        new WaitCommand(1500),
                        new FollowTrajectory(mecanumDrive, RedRightTrajectories.trajectory6),
                        new OpenLeverCommand(lever),
                        new MoveElevatorPercentage(elevator, 0.42),
                        new FollowTrajectory(mecanumDrive, RedRightTrajectories.trajectory7),
                        new FollowTrajectory(mecanumDrive, RedRightTrajectories.centerpark),
                        new InstantCommand(()-> arm.scorePosition()),
                        new WaitCommand(1000),
                        new ResetElevatorCommand(elevator)
                );

        }
    }
}
