// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AutoRunner;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

// Not finished yet

public class PoseEstimator extends SubsystemBase {
    private static PoseEstimator instance = null;

    private PhotonCamera photonCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    private FalconDrivetrain driveTrainSubsytem;
    private Pose2d currentPose = new Pose2d();
    private double[] botPose;

    public static final Map<Integer, Pose3d> targetPoses =
      Map.of(
          1,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(42.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI)),
          2,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(108.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI)),
          3,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI)),
          4,
          new Pose3d(
              Units.inchesToMeters(636.96),
              Units.inchesToMeters(265.74),
              Units.inchesToMeters(27.38),
              new Rotation3d(0.0, 0.0, Math.PI)),
          5,
          new Pose3d(
              Units.inchesToMeters(14.25),
              Units.inchesToMeters(265.74),
              Units.inchesToMeters(27.38),
              new Rotation3d()),
          6,
          new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
              Units.inchesToMeters(18.22),
              new Rotation3d()),
          7,
          new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(108.19),
              Units.inchesToMeters(18.22),
              new Rotation3d()),
          8,
          new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(42.19),
              Units.inchesToMeters(18.22),
              new Rotation3d()));


    // private static final Vector<N7> stateStDevs = VecBuilder.fill(0.05, 0.05,
    // Units.degreesToRadians(5), 0.05, 0.05,
    // 0.05, 0.05);
    // private static final Vector<N5> local =
    // VecBuilder.fill(Units.degreesToRadians(0.01), 0.01, 0.01, 0.01, 0.01);
    // private static final Vector<N3> vision = VecBuilder.fill(0.5, 0.5,
    // Units.degreesToRadians(10));

    private static DifferentialDrivePoseEstimator poseEstimator;
    private AHRS m_gyro = new AHRS(Port.kUSB1);
    private double pitchOffset = 0.0;
    private double previousPipelineTimestamp = 0;

    public PoseEstimator() {
        this.driveTrainSubsytem = FalconDrivetrain.getInstance();
        pitchOffset = m_gyro.getPitch();
        poseEstimator = new DifferentialDrivePoseEstimator(driveTrainSubsytem.m_kinematics,
                m_gyro.getRotation2d(), driveTrainSubsytem.getLeftSideMeters(),
                driveTrainSubsytem.getRightSideMeters(), new Pose2d(),
                VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(5)), // Local measurement standard deviations.
                                                                        // Left encoder, right encoder, gyro.
                VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10))); // Vision measurement standard deviations.
                                                                        // X, Y, and theta.
    }
    // def not scuffed

    public static PoseEstimator getInstance() {
        if (instance == null) {
            instance = new PoseEstimator();
        }
        return instance;
    }

    @Override
    public void periodic() {
        boolean hasTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false);
        if (hasTarget) {
            botPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose")
                    .getDoubleArray(new double[6]);
            Pose2d limePose = new Pose2d(new Translation2d(botPose[0], botPose[1]), new Rotation2d(botPose[5]));

            // scale accuracy by distance to apriltag? (actually maybe not since our regular odom is screwed anyways)
            poseEstimator.addVisionMeasurement(limePose, Timer.getFPGATimestamp() - (botPose[6] / 1000.0));
        }

        // // Add apriltag through photonlib
        // var pipelineResult = photonCamera.getLatestResult();
        // var resultTimestamp = pipelineResult.getTimestampSeconds();
        // if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
        //     previousPipelineTimestamp = resultTimestamp;
        //     var target = pipelineResult.getBestTarget();
        //     var fiducialId = target.getFiducialId();
        //     if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && fiducialId < targetPoses.size()) {
        //         var targetPose = targetPoses.get(fiducialId);
        //         Transform3d camToTarget = target.getBestCameraToTarget();
        //         Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        //         Pose3d visionMeasurment = camPose.transformBy(camToTarget);
        //         poseEstimator.addVisionMeasurement(visionMeasurment.toPose2d(), resultTimestamp);
        //     }
        // }

        poseEstimator.update(m_gyro.getRotation2d(),
                driveTrainSubsytem.getLeftSideMeters(), driveTrainSubsytem.getRightSideMeters());

        currentPose = poseEstimator.getEstimatedPosition();
        // System.out.println("In pose estimator subsystem");
    }

    public String getFormattedPose() {
        var pose = getCurrentPose();
        return String.format("(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(), pose.getRotation().getDegrees());

    }

    public Pose2d getCurrentPose() {
        return currentPose;
    }

    public double getPitch() {
        // return 0.0;
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
        currentPose = poseEstimator.getEstimatedPosition();
    }

    public void resetHeading() {
        m_gyro.reset();
    }
}