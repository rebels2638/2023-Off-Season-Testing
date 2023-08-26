package frc.robot.subsystems.aprilTagVision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface AprilTagVisionIO {
    @AutoLog
    public static class AprilTagVisionIOInputs {
        public PhotonTrackedTarget frontRightBestTarget;
        public PhotonTrackedTarget frontLeftBestTarget;
        public PhotonTrackedTarget backRightBestTarget;
        public PhotonTrackedTarget backLeftBestTarget;
    }

    public default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
