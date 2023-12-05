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
    public default void setPosition(double goalPositionRad, double currentRadAngle) {
    }
    public default void setVelocity(double goalVelocityRadPerSec, double currentVelocityRadPerSec) {
    }
    public default void configureController(ArmFeedforward pff, PIDController pfb, 
                                                    ArmFeedforward vff, PIDController vfb) {
    }

    public abstract boolean reachedSetpoint(boolean isPositionalControll);
}
