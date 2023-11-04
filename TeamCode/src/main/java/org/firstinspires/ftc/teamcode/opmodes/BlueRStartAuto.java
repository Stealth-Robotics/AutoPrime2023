package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.commands.DriveForwardInchesCommand;
import org.firstinspires.ftc.teamcode.commands.EjectCommand;
import org.firstinspires.ftc.teamcode.commands.TurnToDegreesCommand;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LeverSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDriveSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name = "BlueRightSideStart", preselectTeleOp = "BLUE | Tele-Op")
public class BlueRStartAuto extends StealthOpMode {

    SimpleMecanumDriveSubsystem drive;
    ElevatorSubsystem elevator;
    CameraSubsystem camera;
    LeverSubsystem lever;
    ArmSubsystem arm;
    IntakeSubsystem intake;
    AirplaneSubsystem airplane;

    public void initialize() {
        drive = new SimpleMecanumDriveSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap, Alliance.BLUE);
        arm = new ArmSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        lever = new LeverSubsystem(hardwareMap);
        airplane = new AirplaneSubsystem(hardwareMap);
        register(drive, elevator, camera, lever, intake, arm, airplane);

        // tell the camera, we are starting on a specific side of the field
    }
    @Override
    public Command getAutoCommand() {
        String ConeLocation = camera.getConePos();
        switch (ConeLocation){
            case "left":
                return new SequentialCommandGroup(
                        new InstantCommand(() -> airplane.close()),
                        new DriveForwardInchesCommand(drive,24),
                        new TurnToDegreesCommand(drive,-90),
                        new EjectCommand(intake),
                        new TurnToDegreesCommand(drive,0)
                );
            case "right":
                return new SequentialCommandGroup(
                        new InstantCommand(() -> airplane.close()),
                        new DriveForwardInchesCommand(drive,24),
                        new TurnToDegreesCommand(drive,90),
                        new EjectCommand(intake),
                        new TurnToDegreesCommand(drive,0)
                );

            default:
                return new SequentialCommandGroup(
                        new InstantCommand(() -> airplane.close()),
                        new DriveForwardInchesCommand(drive,24),
                        new EjectCommand(intake)
                );

        }
    }
}
