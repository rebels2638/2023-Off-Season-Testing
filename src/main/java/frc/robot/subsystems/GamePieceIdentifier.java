// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GamePieceIdentifier extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static final Pose3d limelightMountPose = new Pose3d(new Translation3d(1,2,3), new Rotation3d(0, 1, 0)); // TODO: find these!
  private PoseEstimator poseEstimator = PoseEstimator.getInstance();

  private double tv, tx, ty, transformedXDistance, transformedYDistance, angleToRobot, yDistance, xDistance, distanceToRobot;
  private NetworkTable limelightNetworkTable;

  private Translation2d relitiveGamePiecePose = new Translation2d(0, 0);
  private Translation2d  absoluteGamePiecePose = poseEstimator.getCurrentPose().getTranslation();


  public GamePieceIdentifier() {
    limelightNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
    distanceToRobot = 0;
  }

  @Override
  public void periodic() {
    
    tv = limelightNetworkTable.getEntry("tv").getDouble(0);
    tx = Math.toRadians(limelightNetworkTable.getEntry("tx").getDouble(0));
    ty = Math.toRadians(limelightNetworkTable.getEntry("ty").getDouble(0));
    
    // not sure if tv is only 1 and 0
    // heres is the articel descrining these calculations : https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
    if (tv > 0) {
      //relitive to robot
      yDistance = Math.tan(ty + limelightMountPose.getRotation().getY()) * limelightMountPose.getTranslation().getZ();
      xDistance = yDistance / Math.tan(tx + limelightMountPose.getRotation().getX());
      angleToRobot = poseEstimator.getCurrentPose().getRotation().getRadians() - tx;
      //double check rot
      relitiveGamePiecePose = new Translation2d(xDistance, yDistance);
      absoluteGamePiecePose = new Translation2d(Math.cos(angleToRobot) * distanceToRobot  + 
                                                  poseEstimator.getCurrentPose().getX(), 
                                                  Math.sin(angleToRobot) * distanceToRobot + poseEstimator.getCurrentPose().getY());
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Translation2d GetRelitiveGamePieceTranslation() {
      return relitiveGamePiecePose;
  }
  public Translation2d GetAbsoluteGamePieceTranslation() {
    return absoluteGamePiecePose;
  }
  public double GetAngleToRobot() {
    return angleToRobot;
  }
  public double GetDistanceFromGamePiece(){
    return distanceToRobot;
  }
  public boolean GamePieceVisible() {
    if (tv > 0) {
      return true;
    }
    return false;
  }

}
