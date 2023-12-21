package org.firstinspires.ftc.teamcode.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class BlueLeftTrajectories {
    public static Pose2d startingPose = new Pose2d(11,60,Math.toRadians(270));

    public static Trajectory scorepixelcenter = TrajectoryBuilder.buildTrajectory(startingPose)
            .forward(29)
            .build();
    public static Trajectory trajectory2 = TrajectoryBuilder.buildTrajectory(scorepixelcenter.end())
            .back(4)
            .build();
    public static Trajectory trajectory3 = TrajectoryBuilder.buildTrajectory(trajectory2.end())
            .lineToSplineHeading(new Pose2d(44, 35, Math.toRadians(180)))
            .build();
    public static Trajectory scorepixelleft = TrajectoryBuilder.buildTrajectory(startingPose)
            .splineTo(new Vector2d(31, 33), Math.toRadians(182))
            .build();
    public static Trajectory scorepixelright = TrajectoryBuilder.buildTrajectory(startingPose)
            .splineTo(new Vector2d(11, 32), Math.toRadians(184))
            .build();
    public static Trajectory trajectory4 = TrajectoryBuilder.buildTrajectory(scorepixelright.end())
            .back(36)
            .build();
    public static Trajectory trajectory5 = TrajectoryBuilder.buildTrajectory(scorepixelleft.end())
            .lineToSplineHeading(new Pose2d(52, 35, Math.toRadians(-180)))
            .build();
}
