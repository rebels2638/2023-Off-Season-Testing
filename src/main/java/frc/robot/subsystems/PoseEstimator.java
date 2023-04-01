// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;

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
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AutoRunner;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

// Not finished yet

public class PoseEstimator extends SubsystemBase {
    private static PoseEstimator instance = null;

    // private PhotonCamera photonCamera = new PhotonCamera("camera");
    private FalconDrivetrain driveTrainSubsytem;
    private Pose2d currentPose = new Pose2d();
    private double[] limelightResult;

    // private static final List<Pose3d> targetPoses = Collections
    //         .unmodifiableList(List.of(new Pose3d(0, 0, 0, new Rotation3d(0, 0, degreesToRadians(180))),
    //                 (new Pose3d(0, 0, 0, new Rotation3d(0, 0, degreesToRadians(180))))));

    // private static final Vector<N7> stateStDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05,
    //         0.05, 0.05);
    // private static final Vector<N5> local = VecBuilder.fill(Units.degreesToRadians(0.01), 0.01, 0.01, 0.01, 0.01);
    // private static final Vector<N3> vision = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    private static DifferentialDrivePoseEstimator poseEstimator;
    private AHRS m_gyro = new AHRS(Port.kUSB1);
	private double pitchOffset = 0.0;
    // private double previousPipelineTimestamp = 0;

    public PoseEstimator() {
        this.driveTrainSubsytem = FalconDrivetrain.getInstance();
        pitchOffset = m_gyro.getPitch();
        poseEstimator = new DifferentialDrivePoseEstimator(driveTrainSubsytem.m_kinematics,
                m_gyro.getRotation2d(), driveTrainSubsytem.getLeftSideMeters(),
                driveTrainSubsytem.getRightSideMeters(), new Pose2d(),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations.
                                                                             // Left encoder, right encoder, gyro.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations.
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

        //         var visionMeasurment = camPose.transformBy(camToTarget); // May be an issue here
        //         poseEstimator.addVisionMeasurement(visionMeasurment.toPose2d(), resultTimestamp);
        //     }
        // }
        double hasTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        if ( hasTarget !=  0) {
            limelightResult = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
            currentPose = new Pose2d(limelightResult[0] , limelightResult[1], m_gyro.getRotation2d());
        }
        else { 
            poseEstimator.update(m_gyro.getRotation2d(),
                    driveTrainSubsytem.getLeftSideMeters(), driveTrainSubsytem.getRightSideMeters());

            currentPose = poseEstimator.getEstimatedPosition();
        }

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

    public double getAngle() {
        return m_gyro.getAngle();
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
        setYawAdjustment(pose.getRotation().getDegrees() - getAngle());
        poseEstimator.resetPosition(pose.getRotation(), driveTrainSubsytem.getLeftSideMeters(), driveTrainSubsytem.getRightSideMeters(), pose);
        currentPose = poseEstimator.getEstimatedPosition();
    }

    public void resetHeading() {
        m_gyro.reset();
    }
}