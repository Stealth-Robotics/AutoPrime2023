package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Trajectories.BlueLeftTrajectories;
import org.firstinspires.ftc.teamcode.Trajectories.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.commands.CloseLeverCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.LeverSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PreloadHolder;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDriveSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.commands.WaitBeforeCommand;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name = "BlueLeftSideStart", preselectTeleOp = "BLUE | Tele-Op")
public class BlueLStartAuto extends StealthOpMode {

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
        camera = new CameraSubsystem(hardwareMap, Alliance.BLUE);
        arm = new ArmSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        lever = new LeverSubsystem(hardwareMap);
        airplane = new AirplaneSubsystem(hardwareMap);
        preload = new PreloadHolder(hardwareMap);
        register(drive, elevator, camera, lever, intake, arm, airplane, preload);

        schedule(new ResetElevatorCommand(elevator));

        // tell the camera, we are starting on a specific side of the field
    }

    @Override
    public void whileWaitingToStart() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public Command getAutoCommand() {
        String ConeLocation = camera.getConePos();
        drive.setPoseEstimate(11, 60, Math.toRadians(270));
        //guesses center for now because the angle and camera don't work rn
        ConeLocation = "center";
        switch (ConeLocation) {
            case "left":
                return new SequentialCommandGroup(
                        new InstantCommand(() -> preload.close()),
                        new FollowTrajectory(mecanumDrive, BlueLeftTrajectories.scorepixelleft),
                        new InstantCommand(() -> preload.open()),
                        new FollowTrajectory(mecanumDrive, BlueLeftTrajectories.trajectory5)

                );
            case "right":
                return new SequentialCommandGroup(
                        new InstantCommand(() -> preload.close()),
                        new FollowTrajectory(mecanumDrive, BlueLeftTrajectories.scorepixelright),
                        new InstantCommand(() -> preload.open()),
                        new FollowTrajectory(mecanumDrive, BlueLeftTrajectories.trajectory4)

                );

            default:
                return new SequentialCommandGroup(

                        new InstantCommand(()-> preload.close()),
                        new InstantCommand(()-> lever.close()),
                        new FollowTrajectory(mecanumDrive, BlueLeftTrajectories.scorepixelcenter),
                        new InstantCommand(()-> preload.open()),
                        new FollowTrajectory(mecanumDrive, BlueLeftTrajectories.trajectory2),
                        new FollowTrajectory(mecanumDrive, BlueLeftTrajectories.trajectory3),
                        new MoveElevatorPercentage(elevator, 0.38),
                        new InstantCommand(()-> arm.intakePosition()),
                        new WaitCommand(3500),
                        new FollowTrajectory(mecanumDrive, BlueLeftTrajectories.trajectory6),
                        new OpenLeverCommand(lever),
                        new MoveElevatorPercentage(elevator, 0.42),
                        new FollowTrajectory(mecanumDrive, BlueLeftTrajectories.trajectory7),
                        new FollowTrajectory(mecanumDrive, BlueLeftTrajectories.centerpark),
                        new InstantCommand(()-> arm.scorePosition()),
                        new WaitCommand(500),
                        new CloseLeverCommand(lever),
                        new ResetElevatorCommand(elevator)
                );

        }
    }
}
