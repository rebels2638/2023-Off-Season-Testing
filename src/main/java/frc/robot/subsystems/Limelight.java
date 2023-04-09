package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private static Limelight instance = null;

    private PhotonCamera photonCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    private AprilTagFieldLayout aprilTagFieldLayout;
    private Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    private PhotonPoseEstimator photonPoseEstimator;

    /** Vision processing for Limelight using Photonlib. */
    public Limelight() {
        try {
            aprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        } catch (IOException e) {
            e.printStackTrace();
        }
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);    
    }
    
    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

    public EstimatedRobotPose getEstimatedPose(Pose3d referencePose) {
        photonPoseEstimator.setReferencePose(referencePose);
        Optional<EstimatedRobotPose> result = photonPoseEstimator.update();
        if(result.isPresent()) {
            EstimatedRobotPose photonPose = result.get();
            return photonPose;
        }

        return null;
    }

    public void setLED(boolean on) {
        photonCamera.setLED(on ? VisionLEDMode.kOn : VisionLEDMode.kOff);
    }

    public void setDriverMode(boolean on) {
        photonCamera.setDriverMode(on);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }
}
