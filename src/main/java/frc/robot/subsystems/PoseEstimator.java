// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.AutoConstants.LimelightConstants;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PoseEstimator extends SubsystemBase {
    private static PoseEstimator instance = null;

    private FalconDrivetrain driveTrainSubsytem;
    private Limelight limelight;
    private Pose2d currentPose = new Pose2d();
    private double[] botPose;

    private static DifferentialDrivePoseEstimator poseEstimator;
    private AHRS m_gyro = new AHRS(Port.kUSB1);
    private double pitchOffset = 0.0;
    private double previousPipelineTimestamp = 0;

    public PoseEstimator() {
        this.driveTrainSubsytem = FalconDrivetrain.getInstance();
        this.limelight = Limelight.getInstance();
        pitchOffset = m_gyro.getPitch();
        poseEstimator = new DifferentialDrivePoseEstimator(driveTrainSubsytem.m_kinematics,
                m_gyro.getRotation2d(), driveTrainSubsytem.getLeftSideMeters(),
                driveTrainSubsytem.getRightSideMeters(), new Pose2d(),
                VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(5)), // State measurement standard deviations.
                                                                        // Left encoder, right encoder, gyro.
                VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10))); // Vision measurement standard deviations.
                                                                        // X, Y, and theta
    }

    public static PoseEstimator getInstance() {
        if (instance == null) {
            instance = new PoseEstimator();
        }
        return instance;
    }

    @Override
    public void periodic() {
        poseEstimator.update(m_gyro.getRotation2d(),
                driveTrainSubsytem.getLeftSideMeters(), driveTrainSubsytem.getRightSideMeters());

        // Limelight vision pose estimates
        // double hasTarget =
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0.0);
        // if (hasTarget != -1.0) {
        // if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        // botPose =
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue")
        // .getDoubleArray(new double[6]);
        // } else {
        // botPose =
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired")
        // .getDoubleArray(new double[6]);
        // }

        // Pose2d limePose = new Pose2d(new Translation2d(botPose[0], botPose[1]),
        // m_gyro.getRotation2d());

        // double latency =
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0.0)
        // +
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("cl").getDouble(0.0);
        // poseEstimator.addVisionMeasurement(limePose, Timer.getFPGATimestamp() -
        // (latency / 1000.0));
        // }

        // Add apriltag pose estimates through photonlib
        if (limelight.getMode() == LimelightConstants.APRILTAG_PIPELINE) {
            currentPose = poseEstimator.getEstimatedPosition();
            EstimatedRobotPose photonPose = limelight.getEstimatedPose(new Pose3d(currentPose));
            if (photonPose != null)
                poseEstimator.addVisionMeasurement(photonPose.estimatedPose.toPose2d(), photonPose.timestampSeconds);
        }

        currentPose = poseEstimator.getEstimatedPosition();
    }

    public String getFormattedPose() {
        var pose = getCurrentPose();
        return String.format("(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    public Pose2d getCurrentPose() {
        return currentPose;
    }

    public double getPitch() {
        return m_gyro.getPitch() - pitchOffset;
    }

    public double getGyroAngle() {
        return m_gyro.getAngle();
    }

    public double getYaw() {
        return currentPose.getRotation().getRadians();
    }

    public void resetPitchOffset() {
        pitchOffset = m_gyro.getPitch();
    }

    public void setYawAdjustment(double deg) {
        m_gyro.setAngleAdjustment(deg);
    }

    public static double degreesToRadians(int degrees) {
        return (degrees / 180.0) * Math.PI;
    }

    public void resetPose(Pose2d pose) {
        FalconDrivetrain.getInstance().zeroEncoder();
        setYawAdjustment(pose.getRotation().getDegrees() - getGyroAngle());
        poseEstimator.resetPosition(pose.getRotation(), driveTrainSubsytem.getLeftSideMeters(),
                driveTrainSubsytem.getRightSideMeters(), pose);
        limelight.setMode(LimelightConstants.APRILTAG_PIPELINE);
        currentPose = poseEstimator.getEstimatedPosition();
    }

    public void resetHeading() {
        m_gyro.reset();
    }
}