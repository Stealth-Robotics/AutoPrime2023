package org.firstinspires.ftc.teamcode.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class BlueRightTrajectories {
    public static Pose2d startingPose = new Pose2d(-35,61,Math.toRadians(270));

    public static Trajectory scorepixelcenter = TrajectoryBuilder.buildTrajectory(startingPose)
            .forward(29)
            .build();
    public static Trajectory trajectory2 = TrajectoryBuilder.buildTrajectory(scorepixelcenter.end())
            .back(4)
            .build();
    public static Trajectory trajectory3 = TrajectoryBuilder.buildTrajectory(trajectory2.end())
            .lineToConstantHeading(new Vector2d(-55, 36))
            .build();
    public static Trajectory trajectory3part2 = TrajectoryBuilder.buildTrajectory(trajectory3.end())
            .lineToLinearHeading(new Pose2d(-55, 10, Math.toRadians(0)))
            .build();
    public static Trajectory trajectory3part3 = TrajectoryBuilder.buildTrajectory(trajectory3part2.end())
            .lineToConstantHeading(new Vector2d(35, 10))
            .build();
    public static Trajectory trajectory3part4 = TrajectoryBuilder.buildTrajectory(trajectory3part3.end())
            .lineToLinearHeading(new Pose2d(48, 35, Math.toRadians(180)))
            .build();
    public static Trajectory scorepixelleft = TrajectoryBuilder.buildTrajectory(startingPose)
            .splineTo(new Vector2d(30, 33), Math.toRadians(182),
                    SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(10))
            .build();
    public static Trajectory scorepixelright = TrajectoryBuilder.buildTrajectory(startingPose)
            .splineTo(new Vector2d(8, 30), Math.toRadians(186),
                    SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(10))
            .build();
    public static Trajectory trajectory4 = TrajectoryBuilder.buildTrajectory(scorepixelright.end())
            .lineToLinearHeading(new Pose2d(46, 28, Math.toRadians(186)))
            .build();
    public static Trajectory trajectory5 = TrajectoryBuilder.buildTrajectory(scorepixelleft.end())
            .lineToSplineHeading(new Pose2d(52, 35, Math.toRadians(-180)))
            .build();
    public static Trajectory trajectory6 = TrajectoryBuilder.buildTrajectory(trajectory3.end())
            .back(8.5)
            .build();
    public static Trajectory trajectory7 = TrajectoryBuilder.buildTrajectory(trajectory6.end())
            .forward(5)
            .build();
    public static Trajectory centerpark = TrajectoryBuilder.buildTrajectory(trajectory7.end())
            .splineTo(new Vector2d(59, 60), Math.toRadians(6))
            .build();
    public static Trajectory trajectory8 = TrajectoryBuilder.buildTrajectory(trajectory4.end())
            .back(7)
            .build();
    public static Trajectory trajectory9 = TrajectoryBuilder.buildTrajectory(trajectory8.end())
            .forward(5)
            .build();
    public static Trajectory rightpark = TrajectoryBuilder.buildTrajectory(trajectory9.end())
            .splineTo(new Vector2d(59, 60), Math.toRadians(6))
            .build();
    public static Trajectory trajectory10 = TrajectoryBuilder.buildTrajectory(scorepixelleft.end())
            .lineToLinearHeading(new Pose2d(46, 37.8, Math.toRadians(180)))
            .build();
    public static Trajectory trajectory11 = TrajectoryBuilder.buildTrajectory(trajectory10.end())
            .back(7)
            .build();
    public static Trajectory trajectory12 = TrajectoryBuilder.buildTrajectory(trajectory11.end())
            .forward(5)
            .build();
    public static Trajectory leftpark = TrajectoryBuilder.buildTrajectory(trajectory12.end())
            .splineTo(new Vector2d(59, 60), Math.toRadians(6))
            .build();
}
