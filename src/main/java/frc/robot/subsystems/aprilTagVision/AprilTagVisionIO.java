package frc.robot.subsystems.aprilTagVision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;

public interface AprilTagVisionIO {
    @AutoLog
    public static class AprilTagVisionIOInputs {
        public PhotonTrackedTarget frontRightBestTarget;
        public PhotonTrackedTarget frontLeftBestTarget;
        public PhotonTrackedTarget backRightBestTarget;
        public PhotonTrackedTarget backLeftBestTarget;

        public double frontRightPipleineLatency;
        public double frontLeftPipleineLatency;
        public double backRightPipleineLatency;
        public double backLeftPipleineLatency;
    }

    public default void updateInputs(AprilTagVisionIOInputs inputs, Pose3d refrencePose) {}
}
