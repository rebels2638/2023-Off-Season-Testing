package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.SimVisionSystem;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.AutoConstants.LimelightConstants;

public class Limelight extends SubsystemBase {
    private static Limelight instance = null;

    private PhotonCamera photonCamera = new PhotonCamera(LimelightConstants.CAMERA_NAME);
    private PhotonPoseEstimator photonPoseEstimator;

    private int mode = 0;

    private double camDiagFOV = 75.26; // degrees - assume wide-angle camera
    private double maxLEDRange = 20; // meters
    private int camResolutionWidth = 960; // pixels
    private int camResolutionHeight = 720; // pixels
    private double minTargetArea = 10; // square pixels

    private SimVisionSystem simVision;

    /** Vision processing for Limelight using Photonlib. */
    public Limelight() {
        photonPoseEstimator = new PhotonPoseEstimator(LimelightConstants.aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP, photonCamera, LimelightConstants.ROBOT_TO_CAM_TRANSFORM);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        setPipeline(LimelightConstants.DEFAULT_PIPELINE);

        if(Robot.isSimulation()) {
            simVision = new SimVisionSystem(
                LimelightConstants.CAMERA_NAME,
                camDiagFOV,
                LimelightConstants.ROBOT_TO_CAM_TRANSFORM,
                maxLEDRange,
                camResolutionWidth,
                camResolutionHeight,
                minTargetArea);
            simVision.addVisionTargets(LimelightConstants.aprilTagFieldLayout);
        }
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
        if (result.isPresent()) {
            EstimatedRobotPose photonPose = result.get();
            return photonPose;
        }

        return null;
    }

    public PhotonTrackedTarget getLatestTarget() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget();
        }

        return null;
    }

    private void setLED(boolean on) {
        photonCamera.setLED(on ? VisionLEDMode.kOn : VisionLEDMode.kOff);
    }

    public void setWarningLED() {
        photonCamera.setLED(VisionLEDMode.kBlink);
    }

    private void setDriverMode(boolean on) {
        photonCamera.setDriverMode(on);
    }

    private void setPipeline(int pipelineID) {
        photonCamera.setPipelineIndex(pipelineID);
    }

    public void setMode(int pipelineID) {
        mode = pipelineID;
        setPipeline(pipelineID);
        setDriverMode(pipelineID == LimelightConstants.DRIVER_PIPELINE);
        setLED(pipelineID == LimelightConstants.REFLECTIVETAPE_PIPELINE);
    }

    public int getMode() {
        return mode;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        simVision.processFrame(FalconDrivetrain.getInstance().getSimulatedPose());
    }
}
