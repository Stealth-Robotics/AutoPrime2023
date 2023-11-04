package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveForwardInchesCommand;
import org.firstinspires.ftc.teamcode.commands.EjectCommand;
import org.firstinspires.ftc.teamcode.commands.TurnToDegreesCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LeverSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDriveSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name = "RedLeftSideStart", preselectTeleOp = "RED | Tele-Op")
public class RedLStartAuto extends StealthOpMode {

    SimpleMecanumDriveSubsystem drive;
    ElevatorSubsystem elevator;
    CameraSubsystem camera;
    LeverSubsystem lever;
    ArmSubsystem arm;
    IntakeSubsystem intake;

    public void initialize() {
        drive = new SimpleMecanumDriveSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap, Alliance.RED);
        arm = new ArmSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        lever = new LeverSubsystem(hardwareMap);
        register(drive, elevator, camera, lever, intake, arm);

        // tell the camera, we are starting on a specific side of the field
    }
    @Override
    public Command getAutoCommand() {
        String ConeLocation = camera.getConePos();
        switch (ConeLocation){
            case "left":
                return new SequentialCommandGroup(
                        new DriveForwardInchesCommand(drive,24),
                        new TurnToDegreesCommand(drive,-90),
                        new EjectCommand(intake),
                        new TurnToDegreesCommand(drive,0)
                );
            case "right":
                return new SequentialCommandGroup(
                        new DriveForwardInchesCommand(drive,24),
                        new TurnToDegreesCommand(drive,90),
                        new EjectCommand(intake),
                        new TurnToDegreesCommand(drive,0)
                );

            default:
                return new SequentialCommandGroup(
                        new DriveForwardInchesCommand(drive,24),
                        new EjectCommand(intake)
                );

        }
    }
}
