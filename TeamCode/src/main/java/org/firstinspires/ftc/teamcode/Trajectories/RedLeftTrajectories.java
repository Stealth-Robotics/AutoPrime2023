package org.firstinspires.ftc.teamcode.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class RedLeftTrajectories {
    public static Pose2d startingPose = new Pose2d(-31,-64.5,Math.toRadians(90));

    public static Trajectory trajectory1 = TrajectoryBuilder.buildTrajectory(startingPose)
            .forward(28)
            .build();
}
