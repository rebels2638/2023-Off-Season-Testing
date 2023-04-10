package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.AutoConstants.LimelightConstants;

public class Limelight extends SubsystemBase {
    private static Limelight instance = null;

    private PhotonCamera photonCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    private PhotonPoseEstimator photonPoseEstimator;

    /** Vision processing for Limelight using Photonlib. */
    public Limelight() {
        photonPoseEstimator = new PhotonPoseEstimator(LimelightConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, LimelightConstants.ROBOT_TO_CAM_TRANSFORM);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        setPipeline(LimelightConstants.DEFAULT_PIPELINE);
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

    public PhotonTrackedTarget getLatestTarget() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if(result.hasTargets()) {
            return result.getBestTarget();
        }

        return null;
    }

    private void setLED(boolean on) {
        photonCamera.setLED(on ? VisionLEDMode.kOn : VisionLEDMode.kOff);
    }

    private void setWarningLED() {
        photonCamera.setLED(VisionLEDMode.kBlink);
    }

    private void setDriverMode(boolean on) {
        photonCamera.setDriverMode(on);
    }

    private void setPipeline(int pipelineID) {
        photonCamera.setPipelineIndex(pipelineID);
    }

    public void setMode(int pipelineID) {
        setPipeline(pipelineID);
        setDriverMode(pipelineID == LimelightConstants.DRIVER_PIPELINE);
        setLED(pipelineID == LimelightConstants.REFLECTIVETAPE_PIPELINE);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        System.out.println(LimelightConstants.CAM_TO_ARM_DIST);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }
}
