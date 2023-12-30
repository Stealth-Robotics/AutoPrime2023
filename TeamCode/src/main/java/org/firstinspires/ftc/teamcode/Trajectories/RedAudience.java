package org.firstinspires.ftc.teamcode.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class RedAudience {
    public static Pose2d startingPose = new Pose2d(-38.5,-61,Math.toRadians(90));

    public static Trajectory trajectory1 = TrajectoryBuilder.buildTrajectory(startingPose)
            .lineToLinearHeading(new Pose2d(-38.5, -8.8, Math.toRadians(0)),
                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(40))
            .build();
    public static Trajectory trajectory2 = TrajectoryBuilder.buildTrajectory(trajectory1.end())
            .forward(24,
                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(40))
            .build();
}
