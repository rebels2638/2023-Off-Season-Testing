package frc.robot.utils;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.0;

    // public static final TrajectoryConstraint autoVoltageConstraint = new
    // TrajectoryConstraint();

    public static final double kRamseteB = 1.0;
    public static final double kRamseteZeta = 1.0;

    public static final Map<String, Command> AUTO_EVENT_MAP = new HashMap<String, Command>();

    public static final Pose3d ROBOT_TO_ARM_POSE = new Pose3d(new Translation3d(1.2446, -0.0381, 0.0), new Rotation3d());
    public static final double CONE_TARGET_HEIGHT = 1.1128375;

    public static final class LimelightConstants {
        public static AprilTagFieldLayout aprilTagFieldLayout;
        public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(new Translation3d(0.0, 0.0, 1.3),
                new Rotation3d(0, -10, 0));
        public static final Transform3d ROBOT_TO_CAM_TRANSFORM_UNTILTED = new Transform3d(ROBOT_TO_CAM_TRANSFORM.getTranslation(),
                new Rotation3d());
        public static final Pose2d CAM_TO_ARM_POSE = ROBOT_TO_ARM_POSE.transformBy(ROBOT_TO_CAM_TRANSFORM_UNTILTED).toPose2d();
        public static final double CAM_TO_ARM_YAW = Math.atan2(CAM_TO_ARM_POSE.getY(), CAM_TO_ARM_POSE.getX());
        public static final double CAM_TO_ARM_DIST = CAM_TO_ARM_POSE.getTranslation().getNorm();

        public static final int DRIVER_PIPELINE = 0;
        public static final int APRILTAG_PIPELINE = 1;
        public static final int REFLECTIVETAPE_PIPELINE = 2;
        public static final int CONE_PIPELINE = 3;
        public static final int CUBE_PIPELINE = 4;

        public static final int DEFAULT_PIPELINE = APRILTAG_PIPELINE;

        LimelightConstants() {
            try {
                aprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public static class PoseMap {
        public static final Map<Integer, Pose3d> apriltagPoses = Map.of(
                1,
                new Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(42.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d(0.0, 0.0, Math.PI)),
                2,
                new Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(108.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d(0.0, 0.0, Math.PI)),
                3,
                new Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                        Units.inchesToMeters(18.22),
                        new Rotation3d(0.0, 0.0, Math.PI)),
                4,
                new Pose3d(
                        Units.inchesToMeters(636.96),
                        Units.inchesToMeters(265.74),
                        Units.inchesToMeters(27.38),
                        new Rotation3d(0.0, 0.0, Math.PI)),
                5,
                new Pose3d(
                        Units.inchesToMeters(14.25),
                        Units.inchesToMeters(265.74),
                        Units.inchesToMeters(27.38),
                        new Rotation3d()),
                6,
                new Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                        Units.inchesToMeters(18.22),
                        new Rotation3d()),
                7,
                new Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(108.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d()),
                8,
                new Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(42.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d()));
    }

}
