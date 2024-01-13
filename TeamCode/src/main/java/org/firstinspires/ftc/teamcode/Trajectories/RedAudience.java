package org.firstinspires.ftc.teamcode.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class RedAudience {
    public static Pose2d startingPose = new Pose2d(-38.5,-61,Math.toRadians(90));

    public static Trajectory scorepixelcenter = TrajectoryBuilder.buildTrajectory(startingPose)
            .lineToLinearHeading(new Pose2d(-34, -31, Math.toRadians(90)),
                    SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(20))
            .build();

    public static Trajectory centerbackup = TrajectoryBuilder.buildTrajectory(scorepixelcenter.end())
            .back(4)
            .build();
    public static Trajectory scorepixelleft = TrajectoryBuilder.buildTrajectory(startingPose)
            .lineToConstantHeading(new Vector2d(-40, -37),
                    SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(10))
            .build();

    public static Trajectory scorepixelright = TrajectoryBuilder.buildTrajectory(startingPose)
            .lineToLinearHeading(new Pose2d(-37, -33, Math.toRadians(0)),
                    SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(10))
            .build();
    public static Trajectory rightforward = TrajectoryBuilder.buildTrajectory(scorepixelright.end())
            .forward(4)
            .build();
    public static Trajectory leftbackup = TrajectoryBuilder.buildTrajectory(scorepixelleft.end())
            .back(4)
            .build();
}
