package frc.robot.subsystems.swerve;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.Constants.SwerveConstants;

public class SwerveDrivetrain extends SubsystemBase {

  // Creating my kinematics object using the module locations
  private static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    SwerveConstants.m_frontLeftLocation, SwerveConstants.m_frontRightLocation,
    SwerveConstants.m_backLeftLocation, SwerveConstants.m_backRightLocation);

  // Front left module state
  private static SwerveModuleState frontLeftState;
  // Front right module state
  private static SwerveModuleState frontRightState;
  // Back left module state
  private static SwerveModuleState backLeftState;
  // Back right module state
  private static SwerveModuleState backRightState;

  private ChassisSpeeds speeds;
  
  /** Creates a new ExampleSubsystem. */
  public SwerveDrivetrain() {
  }

  @Override
  public void periodic() {
    // Convert to module states
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Asign module states
    frontLeftState = moduleStates[0];
    frontRightState = moduleStates[1];
    backLeftState = moduleStates[2];
    backRightState = moduleStates[3];
    
    //pass states into each swerve module

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  /**
   * @param fieldRelativeSpeeds The ChassisSpeeds object representing the speeds in the field frame
   *     of reference. Positive x is away from your alliance wall. Positive y is to your left when
   *     standing behind the alliance wall.
   * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
   *     considered to be zero when it is facing directly away from your alliance station wall.
   *     Remember that this should be CCW positive.
   * @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
   */
  public void fieldOrientedDrive(ChassisSpeeds fieldRelativeSpeeds, Rotation2d robotAngle) {
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, robotAngle);
  }

  
  public void robotRelativeDrive(ChassisSpeeds speeds) {
    this.speeds = speeds;
  }   
}
