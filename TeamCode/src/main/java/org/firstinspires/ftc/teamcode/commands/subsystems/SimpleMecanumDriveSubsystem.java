package org.firstinspires.ftc.teamcode.commands.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.stealthrobotics.library.AutoToTeleStorage;

/**
 * This is the most basic Mecanum subsystem you can have, and provides simple methods to drive and stop.
 */
public class SimpleMecanumDriveSubsystem extends SubsystemBase {
    public final DcMotor leftFrontDrive;
    public final DcMotor leftRearDrive;
    public final DcMotor rightFrontDrive;
    public DcMotor rightRearDrive;
    final IMU imu;
    boolean fieldcentric = true;
    boolean slowMode = false;
    double headingOffset = 0.0;

    public SimpleMecanumDriveSubsystem(HardwareMap hardwareMap) {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );

        imu.initialize(parameters);

    }

    /**
     * Drive using gamepad inputs. This includes compensating for imperfect strafing, and adjusting
     * inputs based on stick directions. Better versions would include field-centric driving,
     * deadbands, and more.
     */
    public void driveTeleop(double leftSickY, double leftStickX, double rightStickX) {
        // This code is pulled from Game Manual 0
        // https://gm0.org/en/latest/docs/software/mecanum-drive.html

        double y = -leftSickY; // Remember, this is reversed!
        double x = leftStickX * 1.1; // Counteract imperfect strafing
        double rotation = rightStickX;
        if (fieldcentric) {
            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -getHeading();
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
            x = rotX;
            y = rotY;
        }
        y = y * Math.abs(y);
        x = x * Math.abs(x);
        rotation = rotation * Math.abs(rotation);

        drive(y, x, rotation);

    }

    public void toggleFieldCentric() {
        fieldcentric = !fieldcentric;
    }

//    public double getHeading() {
//        return -imu.getAngularOrientation().firstAngle;
//    }

    // The actual heading from the IMU, only adjusted so that positive is clockwise
    public double getRawHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    // The heading we'll use to drive the bot, adjusted for an offset which we can set any time
    // we want to correct for gyro drift as we drive.
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;
    }

    // Adjust our heading so the front of the bot is forward, no matter how we've drifted over time.
    public void resetHeading() {
        headingOffset = getRawHeading();
        AutoToTeleStorage.finalAutoHeading = 0;
    }
    public void drive(double y, double x, double rotation) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);
        double leftFrontDrivePower = (y + x + rotation) / denominator;
        double leftRearDrivePower = (y - x + rotation) / denominator;
        double rightFrontDrivePower = (y - x - rotation) / denominator;
        double rightRearDrivePower = (y + x - rotation) / denominator;

        if(slowMode){
            leftFrontDrivePower *= 0.5;
            leftRearDrivePower *= 0.5;
            rightFrontDrivePower *= 0.5;
            rightRearDrivePower *= 0.5;
        }

        leftFrontDrive.setPower(leftFrontDrivePower);
        leftRearDrive.setPower(leftRearDrivePower);
        rightFrontDrive.setPower(rightFrontDrivePower);
        rightRearDrive.setPower(rightRearDrivePower);
    }

    public void stop(){
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
    }

    public int getTicks(){
        return leftFrontDrive.getCurrentPosition();
    }

    @Override
    public void periodic() {
//        telemetry.addData("Drive ticks", getTicks());
  //      telemetry.addData("Field centric driving", fieldcentric);
    //    telemetry.addData("Bot Heading", getHeading());
       // telemetry.addData("leftfrontpower", leftFrontDrive.getPower());
       // telemetry.addData("leftrearpower", leftRearDrive.getPower());
       // telemetry.addData("rightfrontpower", rightFrontDrive.getPower());
       // telemetry.addData("rightrearpower", rightRearDrive.getPower());

    }
    public void toggleSlowMode(){
        slowMode = !slowMode;
    }
}
