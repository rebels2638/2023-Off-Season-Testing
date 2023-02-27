package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoConstants {
    
    public static final double kMaxSpeedMetersPerSecond = 0.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.0;

    // public static final TrajectoryConstraint autoVoltageConstraint = new TrajectoryConstraint();

    public static final double kRamseteB = 1.0;
    public static final double kRamseteZeta = 1.0;

    public static final Map<String, Command> AUTO_EVENT_MAP = new HashMap<String, Command>();
}
