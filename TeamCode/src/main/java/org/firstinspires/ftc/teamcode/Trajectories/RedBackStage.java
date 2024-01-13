package org.firstinspires.ftc.teamcode.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class RedBackStage {
    public static Pose2d startingPose = new Pose2d(15, -61,Math.toRadians(90));

    public static Trajectory scorepixelcenter = TrajectoryBuilder.buildTrajectory(startingPose)
            .forward(30,
                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(40))
            .build();
    public static Trajectory trajectory2 = TrajectoryBuilder.buildTrajectory(scorepixelcenter.end())
            .back(4,
                    SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(80))
            .build();
    public static Trajectory trajectory3 = TrajectoryBuilder.buildTrajectory(trajectory2.end())
            .lineToLinearHeading(new Pose2d(44, -36, Math.toRadians(174)),
                    SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(80))
            .build();
    public static Trajectory scorepixelleft = TrajectoryBuilder.buildTrajectory(startingPose)
            .lineToLinearHeading(new Pose2d(16, -24, Math.toRadians(176)),
                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(40))
            .build();
    public static Trajectory scorepixelleft2 = TrajectoryBuilder.buildTrajectory(scorepixelleft.end())
            .forward(5,
                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(40))
            .build();
    public static Trajectory scorepixelright = TrajectoryBuilder.buildTrajectory(startingPose)
            .lineToLinearHeading(new Pose2d(30, -36, Math.toRadians(90)),
                    SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(50))
            .build();
    public static Trajectory trajectory4 = TrajectoryBuilder.buildTrajectory(scorepixelright.end())
            .lineToLinearHeading(new Pose2d(44, -43, Math.toRadians(173)),
                    SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(80))
            .build();
//    public static Trajectory trajectory5 = TrajectoryBuilder.buildTrajectory(scorepixelleft2.end())
//            .lineToSplineHeading(new Pose2d(46, -33, Math.toRadians(175)),
//                    SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                    SampleMecanumDrive.getAccelerationConstraint(80))
//            .build();
    public static Trajectory trajectory6 = TrajectoryBuilder.buildTrajectory(trajectory3.end())
            .back(9,
                    SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(80))
            .build();
    public static Trajectory trajectory7 = TrajectoryBuilder.buildTrajectory(trajectory6.end())
            .forward(6,
                    SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(80))
            .build();
    public static Trajectory centerpark = TrajectoryBuilder.buildTrajectory(trajectory7.end())
            .splineToLinearHeading(new Pose2d(50, -60, Math.toRadians(357)), Math.toRadians(6),
                    SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(60))
            .build();
    public static Trajectory centerpark2 = TrajectoryBuilder.buildTrajectory(centerpark.end())
            .forward(10,
                    SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(60))
            .build();
    public static Trajectory trajectory8 = TrajectoryBuilder.buildTrajectory(trajectory4.end())
            .back(9,
                    SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(80))
            .build();
    public static Trajectory trajectory9 = TrajectoryBuilder.buildTrajectory(trajectory8.end())
            .forward(6,
                    SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(80))
            .build();
    public static Trajectory rightpark = TrajectoryBuilder.buildTrajectory(trajectory9.end())
            .splineToLinearHeading(new Pose2d(50, -60, Math.toRadians(357)), Math.toRadians(6),
                    SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(80))
            .build();
    public static Trajectory rightpark2 = TrajectoryBuilder.buildTrajectory(rightpark.end())
            .forward(10,
                    SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(80))
            .build();
    public static Trajectory trajectory10 = TrajectoryBuilder.buildTrajectory(scorepixelleft2.end())
            .lineToLinearHeading(new Pose2d(44, -31, Math.toRadians(174)),
                    SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(80))
            .build();
    public static Trajectory trajectory11 = TrajectoryBuilder.buildTrajectory(trajectory10.end())
            .back(9,
                    SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(80))
            .build();
    public static Trajectory trajectory12 = TrajectoryBuilder.buildTrajectory(trajectory11.end())
            .forward(6,
                    SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(80))
            .build();
    public static Trajectory leftpark = TrajectoryBuilder.buildTrajectory(trajectory12.end())
            .splineToLinearHeading(new Pose2d(50, -60, Math.toRadians(357)), Math.toRadians(6),
                    SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(80))
            .build();
    public static Trajectory leftpark2 = TrajectoryBuilder.buildTrajectory(leftpark.end())
            .forward(10,
                    SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(80))
            .build();
}
