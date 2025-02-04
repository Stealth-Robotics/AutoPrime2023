package org.firstinspires.ftc.teamcode.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class BlueAudience {
    public static Pose2d startingPose = new Pose2d(-32,61,Math.toRadians(270));

    public static Trajectory scorepixelcenter = TrajectoryBuilder.buildTrajectory(startingPose)
            .lineToLinearHeading(new Pose2d(-37, 32, Math.toRadians(270)),
                    SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(20))
            .build();
    public static Trajectory centerbackup = TrajectoryBuilder.buildTrajectory(scorepixelcenter.end())
            .back(4)
            .build();
    public static Trajectory scorepixelleft = TrajectoryBuilder.buildTrajectory(startingPose)
            .splineTo(new Vector2d(-33, 28), Math.toRadians(358),
                    SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(10))
            .build();
    public static Trajectory scorepixelright = TrajectoryBuilder.buildTrajectory(startingPose)
            .lineToLinearHeading(new Pose2d(-37, 33, Math.toRadians(182)),
                    SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(10))
            .build();
    public static Trajectory rightbackup = TrajectoryBuilder.buildTrajectory(scorepixelright.end())
            .back(4)
            .build();
    public static Trajectory leftbackup = TrajectoryBuilder.buildTrajectory(scorepixelleft.end())
            .back(4)
            .build();


}
