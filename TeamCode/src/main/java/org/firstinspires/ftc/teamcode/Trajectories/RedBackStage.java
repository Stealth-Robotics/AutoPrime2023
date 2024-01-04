package org.firstinspires.ftc.teamcode.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class RedBackStage {
    public static Pose2d startingPose = new Pose2d(15, -61,Math.toRadians(90));

    public static Trajectory scorepixelcenter = TrajectoryBuilder.buildTrajectory(startingPose)
            .lineToLinearHeading(new Pose2d(13, -31, Math.toRadians(90)),
                    SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(20))
            .build();
    public static Trajectory trajectory2 = TrajectoryBuilder.buildTrajectory(scorepixelcenter.end())
            .back(4,
                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(40))
            .build();
    public static Trajectory trajectory3 = TrajectoryBuilder.buildTrajectory(trajectory2.end())
            .lineToLinearHeading(new Pose2d(44, -36, Math.toRadians(174)))
            .build();
    public static Trajectory scorepixelleft = TrajectoryBuilder.buildTrajectory(startingPose)
            .splineTo(new Vector2d(11, -27), Math.toRadians(180),
                    SampleMecanumDrive.getVelocityConstraint(16, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(16))
            .build();
    public static Trajectory scorepixelright = TrajectoryBuilder.buildTrajectory(startingPose)
            .splineTo(new Vector2d(33, -25), Math.toRadians(180),
                    SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(10))
            .build();
    public static Trajectory scorepixelrightback = TrajectoryBuilder.buildTrajectory(scorepixelright.end())
            .back(4)
            .build();
    public static Trajectory trajectory4 = TrajectoryBuilder.buildTrajectory(scorepixelrightback.end())
            .lineToLinearHeading(new Pose2d(44, -37.8, Math.toRadians(170)))
            .build();
    public static Trajectory trajectory5 = TrajectoryBuilder.buildTrajectory(scorepixelleft.end())
            .lineToSplineHeading(new Pose2d(52, 35, Math.toRadians(-180)))
            .build();
    public static Trajectory trajectory6 = TrajectoryBuilder.buildTrajectory(trajectory3.end())
            .back(12)
            .build();
    public static Trajectory trajectory7 = TrajectoryBuilder.buildTrajectory(trajectory6.end())
            .forward(6)
            .build();
    public static Trajectory centerpark = TrajectoryBuilder.buildTrajectory(trajectory7.end())
            .splineToLinearHeading(new Pose2d(50, -57.5, Math.toRadians(357)), Math.toRadians(6))
            .build();
    public static Trajectory centerpark2 = TrajectoryBuilder.buildTrajectory(centerpark.end())
            .forward(10)
            .build();
    public static Trajectory trajectory8 = TrajectoryBuilder.buildTrajectory(trajectory4.end())
            .back(12)
            .build();
    public static Trajectory trajectory9 = TrajectoryBuilder.buildTrajectory(trajectory8.end())
            .forward(6)
            .build();
    public static Trajectory rightpark = TrajectoryBuilder.buildTrajectory(trajectory9.end())
            .splineToLinearHeading(new Pose2d(50, -57.5, Math.toRadians(357)), Math.toRadians(6))
            .build();
    public static Trajectory rightpark2 = TrajectoryBuilder.buildTrajectory(rightpark.end())
            .forward(10)
            .build();
    public static Trajectory trajectory10 = TrajectoryBuilder.buildTrajectory(scorepixelleft.end())
            .lineToLinearHeading(new Pose2d(44, -28, Math.toRadians(174)))
            .build();
    public static Trajectory trajectory11 = TrajectoryBuilder.buildTrajectory(trajectory10.end())
            .back(12)
            .build();
    public static Trajectory trajectory12 = TrajectoryBuilder.buildTrajectory(trajectory11.end())
            .forward(6)
            .build();
    public static Trajectory leftpark = TrajectoryBuilder.buildTrajectory(trajectory12.end())
            .splineToLinearHeading(new Pose2d(50, -57.5, Math.toRadians(357)), Math.toRadians(6))
            .build();
    public static Trajectory leftpark2 = TrajectoryBuilder.buildTrajectory(leftpark.end())
            .forward(10)
            .build();
}
