package frc.robot.subsystems.Limelight;

import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {
    @AutoLog
    public static class LimelightIOInputs {
        double tx;
        double ty;
        double area;
        double tv;
    }
    public default void updateInputs(LimelightIOInputs inputs) {}
}
