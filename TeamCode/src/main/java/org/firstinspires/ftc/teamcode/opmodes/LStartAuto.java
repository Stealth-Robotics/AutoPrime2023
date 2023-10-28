package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDriveSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name = "LeftSideStart", preselectTeleOp = "BLUE | Tele-Op")
public class LStartAuto extends StealthOpMode {

    SimpleMecanumDriveSubsystem drive;
    ElevatorSubsystem elevator;
    CameraSubsystem camera;
    ClawSubsystem claw;
    ArmSubsystem arm;
    IntakeSubsystem intake;

    public void initialize() {
        drive = new SimpleMecanumDriveSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap, Alliance.RED);
        arm = new ArmSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);
        register(drive, elevator, camera, claw, intake, arm);

        // tell the camera, we are starting on a specific side of the field
    }
}
