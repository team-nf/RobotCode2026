package frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PoseConstants {
    public static final Pose2d START_POSE_RED_1 = new Pose2d(12.95, 3.65, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d START_POSE_RED_2 = new Pose2d(16.11, 1.24, new Rotation2d(Math.toRadians(180)));

    public static final Pose2d RED_WALL_INTAKE_POSE = new Pose2d(15.1, 2.9, new Rotation2d(Math.toRadians(-31)));
    public static final Pose2d RED_SHOOT_NEAR_TRENCH1_POSE = new Pose2d(13.5, 0.95, new Rotation2d(Math.toRadians(159.305)));
    public static final Pose2d RED_SHOOT_NEAR_TRENCH2_POSE = new Pose2d(13.5, 0.95, new Rotation2d(Math.toRadians(159.305)));

    public static final Pose2d START_POSE_BLUE_1 = new Pose2d(1, 1, new Rotation2d());
    public static final Pose2d START_POSE_BLUE_2 = new Pose2d(1, 1, new Rotation2d());

    public static final Pose2d BLUE_WALL_INTAKE_POSE = new Pose2d(2.0, 2.0, new Rotation2d());
    public static final Pose2d BLUE_SHOOT_NEAR_TRENCH1_POSE = new Pose2d(13.5, 0.95, new Rotation2d(Math.toRadians(159.305)));
    public static final Pose2d BLUE_SHOOT_NEAR_TRENCH2_POSE = new Pose2d(13.5, 0.95, new Rotation2d(Math.toRadians(159.305)));


    public static final Pose2d BLUE_HUB_AIM_POSE = new Pose2d(4.61, 4.1, new Rotation2d());
    public static final Pose2d RED_HUB_AIM_POSE = new Pose2d(11.75, 4.1, new Rotation2d());
}
