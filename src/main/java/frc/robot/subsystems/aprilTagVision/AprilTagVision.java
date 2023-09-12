package frc.robot.subsystems.aprilTagVision;


import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class AprilTagVision extends SubsystemBase{

    private AprilTagVisionIO aprilTagVisionIO;
    private AprilTagVisionIOInputsAutoLogged inputs = new AprilTagVisionIOInputsAutoLogged();
    private Pose3d refrencePose = new Pose3d();
    public AprilTagVision(AprilTagVisionIO aprilTagVisionIO) {
        this.aprilTagVisionIO = aprilTagVisionIO;
    }

    @Override
    public void periodic() {
        aprilTagVisionIO.updateInputs(inputs, refrencePose);
        Logger.getInstance().processInputs("AprilTagVision/inputs", inputs);
    }
    public Pose3d getEstimatedPosition(Pose3d refrencePose) {
        this.refrencePose = refrencePose;
        Pose3d robotPoseEst = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
        int valid = 0;

        Logger.getInstance().recordOutput("AprilTagVision/frontRightBestTarget", false);
        Logger.getInstance().recordOutput("AprilTagVision/frontLeftBestTarget", false);
        Logger.getInstance().recordOutput("AprilTagVision/backRightBestTarget", false);
        Logger.getInstance().recordOutput("AprilTagVision/backLeftBestTarget", false);
        if (inputs.frontRightBestTarget != null) {
            Logger.getInstance().recordOutput("AprilTagVision/frontRightBestTarget", true);
            robotPoseEst = getPoseToTarget(robotPoseEst, inputs.frontRightBestTarget, Constants.VisionConstants.FRONT_RIGHT_CAMERA_ROBOT_TO_CAMERA);
            valid++;
        }
        if (inputs.frontLeftBestTarget != null) {
            Logger.getInstance().recordOutput("AprilTagVision/frontLeftBestTarget", true);
            robotPoseEst = getPoseToTarget(robotPoseEst, inputs.frontLeftBestTarget, Constants.VisionConstants.FRONT_LEFT_CAMERA_ROBOT_TO_CAMERA);
            valid++;
        }
        if (inputs.backRightBestTarget != null) {
            Logger.getInstance().recordOutput("AprilTagVision/backRightBestTarget", true);
            robotPoseEst = getPoseToTarget(robotPoseEst, inputs.backRightBestTarget, Constants.VisionConstants.BACK_RIGHT_CAMERA_ROBOT_TO_CAMERA);
            valid++;
        }
        if (inputs.backLeftBestTarget != null) {
            Logger.getInstance().recordOutput("AprilTagVision/backLeftBestTarget", true);
            robotPoseEst = getPoseToTarget(robotPoseEst, inputs.backLeftBestTarget, Constants.VisionConstants.BACK_LEFT_CAMERA_ROBOT_TO_CAMERA);
            valid++;
        }
        if (valid == 0) {
            return null;
        }
        
        robotPoseEst = new Pose3d(
            new Translation3d(
                robotPoseEst.getX() / valid, 
                robotPoseEst.getY() / valid, 
                robotPoseEst.getZ() / valid), 
            new Rotation3d(
                robotPoseEst.getRotation().getX() / valid, 
                robotPoseEst.getRotation().getY() / valid, 
                robotPoseEst.getRotation().getZ() / valid));
        double[] outLog = { robotPoseEst.getX(), robotPoseEst.getY(), robotPoseEst.getZ(),
        Math.toDegrees(robotPoseEst.getRotation().getX()), 
        Math.toDegrees(robotPoseEst.getRotation().getY()), 
        Math.toDegrees(robotPoseEst.getRotation().getZ()) };
        Logger.getInstance().recordOutput("AprilTagVisionMeasurment", outLog);
        return robotPoseEst;
    }
    public double getPiplineLatency() {
        return (inputs.frontRightPipleineLatency + inputs.frontLeftPipleineLatency + inputs.backRightPipleineLatency + inputs.backLeftPipleineLatency) / 4;
    }

    private Pose3d getPoseToTarget(Pose3d robotPoseEst, PhotonTrackedTarget target, Transform3d cameraToTarget) {
        if (target != null) {
            Pose3d tagPose3d = Constants.VisionConstants.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();

            Pose3d robotTagEst = PhotonUtils.estimateFieldToRobotAprilTag(
                cameraToTarget,
                tagPose3d,
                target.getBestCameraToTarget());

            robotPoseEst = new Pose3d(new Translation3d(robotPoseEst.getX() + robotTagEst.getX(), 
                robotPoseEst.getY() + robotTagEst.getY(), robotPoseEst.getZ() + robotTagEst.getZ()), 
                new Rotation3d(robotPoseEst.getRotation().getX() + robotTagEst.getRotation().getX(), 
                robotPoseEst.getRotation().getY() + robotTagEst.getRotation().getY(), 
                robotPoseEst.getRotation().getZ() + robotTagEst.getRotation().getZ()));
            return robotPoseEst;
        }
        return null;
    }
}
