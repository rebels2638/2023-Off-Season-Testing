// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GamePieceIdentifier extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static final double limelightMountHight = 9; //TODO: find hight
  private static final double limelightMountAngle = 50; //TODO: find angle
  private double tv, tx, ty, transformedXDistance, transformedYDistance;
  private NetworkTable limelightNetworkTable;
  public GamePieceIdentifier() {
    limelightNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    
    tv = limelightNetworkTable.getEntry("tv").getDouble(0);
    tx = limelightNetworkTable.getEntry("tx").getDouble(0);
    ty = limelightNetworkTable.getEntry("ty").getDouble(0);

    // not sure if tv is only 1 and 0
    if (tv == 1) {
      //relitive to limelight
      double yDistance = Math.tan(ty - limelightMountAngle) * limelightMountHight;
      double xDistance = Math.tan(tx) * yDistance;

      // TODO: ADD Constants!!!!
      transformedXDistance = xDistance;
      transformedYDistance = yDistance; 
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Translation2d GetGamePieceTranslation2d() {
      return new Translation2d(transformedXDistance, transformedYDistance);
  }
}
