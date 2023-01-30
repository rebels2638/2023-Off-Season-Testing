// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

// Not finished yet


public class PoseEstimatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private PhotonCamera photonCamera = new PhotonCamera("camera");
  private Drivetrain driveTrainSubsytem = new Drivetrain();
  private final GyroSubystem gyro = new GyroSubystem();
   private static final List<Pose3d> targetPoses = Collections.unmodifiableList
   (List.of(new Pose3d(0, 0, 0, new Rotation3d(0, 0, degreesToRadians(180))),
    (new Pose3d(0, 0, 0, new Rotation3d(0, 0, degreesToRadians(180)))))); 
    
    private static final Vector<N7> stateStDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05, 0.05, 0.05);
    private static final Vector<N5> local = VecBuilder.fill(Units.degreesToRadians(0.01), 0.01, 0.01, 0.01, 0.01);
    private static final Vector<N3> vision = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    private static DifferentialDrivePoseEstimator poseEstimator;
    private final Field2d field2d = new Field2d();
    private double previousPipelineTimestamp = 0;
    private final ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    public PoseEstimatorSubsystem(PhotonCamera camera, Drivetrain drive, GyroSubystem gyro){
        this.photonCamera = camera;
        this.driveTrainSubsytem = drive;
        // poseEstimator = new DifferentialDrivePoseEstimator(drive.expose_odom_kinematics(), 
        //                 driveTrainSubsytem.m_gyro.getRotation2d(), 
        //                 driveTrainSubsytem.m_leftEncoder.getDistance(), 
        //                 driveTrainSubsytem.m_rightEncoder.getDistance(), 
        //                 new Pose2d(), 
        //                 local, vision);
        poseEstimator = new DifferentialDrivePoseEstimator(drive.expose_odom_kinematics(), driveTrainSubsytem.m_gyro.getRotation2d(), driveTrainSubsytem.m_leftEncoder.getDistance(), driveTrainSubsytem.m_rightEncoder.getDistance(), new Pose2d(),
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta.

        tab.addString("Pose", this::getFormattedPose).withPosition(0, 0);
        tab.add("Field", field2d).withPosition(2,0);
    }
  //needs to be finished
  
@Override
public void periodic(){
  var pipelineResult = photonCamera.getLatestResult();
  var resultTimestamp = pipelineResult.getTimestampSeconds();
  if(resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()){
      previousPipelineTimestamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();
      var fiducialId = target.getFiducialId();
      if(target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && fiducialId < targetPoses.size()){
          var targetPose = targetPoses.get(fiducialId);
          Transform3d camToTarget = target.getBestCameraToTarget();
          Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
          
          var visionMeasurment = camPose.transformBy(camToTarget); // May be an issue here
          poseEstimator.addVisionMeasurement(visionMeasurment.toPose2d(), resultTimestamp);
      }
  }

  poseEstimator.update(driveTrainSubsytem.m_gyro.getRotation2d(), 
  driveTrainSubsytem.m_leftEncoder.getDistance(), driveTrainSubsytem.m_rightEncoder.getDistance());

  field2d.setRobotPose(getCurrentPose());

}
  private String getFormattedPose(){
      var pose = getCurrentPose();
      return String.format("(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose(){
      return poseEstimator.getEstimatedPosition();
  }

  public static double degreesToRadians(int degrees){
    return (degrees/180) * Math.PI;
  }
}
