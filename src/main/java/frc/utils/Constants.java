package frc.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public static final class SwerveConstants {
        // Locations for the swerve drive modules relative to the robot center.
        public static final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
        public static final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
        public static final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
        public static final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
    }
}
