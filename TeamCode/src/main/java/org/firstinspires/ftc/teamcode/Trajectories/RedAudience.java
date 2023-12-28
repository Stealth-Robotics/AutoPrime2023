package org.firstinspires.ftc.teamcode.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class RedAudience {
    public static Pose2d startingPose = new Pose2d(39,-61,Math.toRadians(270));

    public static Trajectory trajectory1 = TrajectoryBuilder.buildTrajectory(startingPose)
            .forward(28)
            .build();
}
