package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double positionRad;
        public double positionDeg;
    }

    public default void updateInputs(PivotIOInputs inputs) {
        
    }
    public default void setPosition(double positionRad, double currentRadAngle) {
    }

    public default void configureController(ArmFeedforward ff, PIDController fb) {
    }

    public abstract boolean reachedSetpoint();
}
