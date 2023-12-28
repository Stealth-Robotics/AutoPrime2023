package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Trajectories.BlueAudience;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commands.ResetElevatorCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.commands.subsystems.AirplaneSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.LeverSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.PreloadHolder;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name = "BlueAudienceAuto", preselectTeleOp = "BLUE | Tele-Op")
public class BlueAudienceAuto extends StealthOpMode {

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

    }
    @Override
    public void whileWaitingToStart() {
        CommandScheduler.getInstance().run();
    }
    @Override
    public Command getAutoCommand() {
        String ConeLocation = camera.getConePos();
        drive.setPoseEstimate(-38.5, 61, Math.toRadians(270));

        switch (ConeLocation){
            case "left":
                return new SequentialCommandGroup(


                );
            case "right":
                return new SequentialCommandGroup(


                );

            default:
                return new SequentialCommandGroup(
                        new InstantCommand(()-> preload.close()),
                        new InstantCommand(()-> lever.close()),
                        new FollowTrajectory(mecanumDrive, BlueAudience.scorepixelcenter),
                        new InstantCommand(()-> preload.open()),
                        new FollowTrajectory(mecanumDrive, BlueAudience.trajectory2),
                        new FollowTrajectory(mecanumDrive, BlueAudience.trajectory3),
                        new FollowTrajectory(mecanumDrive, BlueAudience.trajectory3part2),
                        new FollowTrajectory(mecanumDrive, BlueAudience.trajectory3part3),
                        new FollowTrajectory(mecanumDrive, BlueAudience.trajectory3part4)
//                        new MoveElevatorPercentage(elevator, 0.38),
//                        new InstantCommand(()-> arm.intakePosition()),
//                        new WaitCommand(3500),
//                        new FollowTrajectory(mecanumDrive, BlueRightTrajectories.trajectory6),
//                        new OpenLeverCommand(lever),
//                        new MoveElevatorPercentage(elevator, 0.42),
//                        new FollowTrajectory(mecanumDrive, BlueRightTrajectories.trajectory7),
//                        new FollowTrajectory(mecanumDrive, BlueRightTrajectories.centerpark),
//                        new InstantCommand(()-> arm.scorePosition()),
//                        new WaitCommand(500),
//                        new CloseLeverCommand(lever),
//                        new ResetElevatorCommand(elevator)
                );

        }
    }
}
