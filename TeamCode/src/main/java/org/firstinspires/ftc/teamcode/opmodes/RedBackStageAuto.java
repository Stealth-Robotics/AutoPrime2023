package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Trajectories.RedBackStage;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commands.MoveElevatorPercentage;
import org.firstinspires.ftc.teamcode.commands.OpenLeverCommand;
import org.firstinspires.ftc.teamcode.commands.ResetElevatorCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LeverSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PreloadHolder;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name = "RedBackStageAuto", preselectTeleOp = "RED | Tele-Op")
public class RedBackStageAuto extends StealthOpMode {

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
                        new FollowTrajectory(mecanumDrive, RedBackStage.scorepixelleft),
                        new InstantCommand(() -> preload.open()),
                        new WaitCommand(200),
                        new FollowTrajectory(mecanumDrive, RedBackStage.trajectory10),
                        new ParallelCommandGroup(
                                new MoveElevatorPercentage(elevator, 0.4),
                                new InstantCommand(()-> arm.intakePosition())
                        ),
                        new WaitCommand(1500),
                        new FollowTrajectory(mecanumDrive, RedBackStage.trajectory11),
                        new InstantCommand(()-> arm.specialPosition()),
                        new WaitCommand(500),
                        new OpenLeverCommand(lever),
                        new MoveElevatorPercentage(elevator, 0.42),
                        new FollowTrajectory(mecanumDrive, RedBackStage.trajectory12),
                        new FollowTrajectory(mecanumDrive, RedBackStage.leftpark),
                        new InstantCommand(()-> arm.scorePosition()),
                        new WaitCommand(1000),
                        new ResetElevatorCommand(elevator)

                );
            case "right":
                return new SequentialCommandGroup(
                        new InstantCommand(() -> preload.close()),
                        new InstantCommand(()-> lever.close()),
                        new FollowTrajectory(mecanumDrive, RedBackStage.scorepixelright),
                        new InstantCommand(() -> preload.open()),
                        new WaitCommand(200),
                        new FollowTrajectory(mecanumDrive, RedBackStage.trajectory4),
                        new ParallelCommandGroup(
                                new MoveElevatorPercentage(elevator, 0.4),
                                new InstantCommand(()-> arm.intakePosition())
                        ),
                        new WaitCommand(1500),
                        new FollowTrajectory(mecanumDrive, RedBackStage.trajectory8),
                        new InstantCommand(()-> arm.specialPosition()),
                        new WaitCommand(500),
                        new OpenLeverCommand(lever),
                        new MoveElevatorPercentage(elevator, 0.42),
                        new FollowTrajectory(mecanumDrive, RedBackStage.trajectory9),
                        new FollowTrajectory(mecanumDrive, RedBackStage.rightpark),
                        new InstantCommand(()-> arm.scorePosition()),
                        new WaitCommand(1000),
                        new ResetElevatorCommand(elevator)
                );

            default:
                return new SequentialCommandGroup(
                        new InstantCommand(()-> preload.close()),
                        new InstantCommand(()-> lever.close()),
                        new FollowTrajectory(mecanumDrive, RedBackStage.scorepixelcenter),
                        new InstantCommand(()-> preload.open()),
                        new FollowTrajectory(mecanumDrive, RedBackStage.trajectory2),
                        new FollowTrajectory(mecanumDrive, RedBackStage.trajectory3),
                        new ParallelCommandGroup(
                                new MoveElevatorPercentage(elevator, 0.42),
                                new InstantCommand(()-> arm.intakePosition())
                        ),
                        new WaitCommand(1500),
                        new FollowTrajectory(mecanumDrive, RedBackStage.trajectory6),
                        new InstantCommand(()-> arm.specialPosition()),
                        new WaitCommand(500),
                        new OpenLeverCommand(lever),
                        new MoveElevatorPercentage(elevator, 0.44),
                        new FollowTrajectory(mecanumDrive, RedBackStage.trajectory7),
                        new FollowTrajectory(mecanumDrive, RedBackStage.centerpark),
                        new FollowTrajectory(mecanumDrive, RedBackStage.centerpark2),
                        new InstantCommand(()-> arm.scorePosition()),
                        new WaitCommand(1000),
                        new ResetElevatorCommand(elevator)
                );

        }
    }
}
