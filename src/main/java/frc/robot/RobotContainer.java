// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.function.DoubleSupplier;


import java.io.File;
import frc.robot.commands.drivetrain.AbsoluteFieldDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.RebelUtil;
import frc.lib.input.XboxController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.AutoConstants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drivetrain.AbsoluteDrive;

import frc.robot.commands.drivetrain.DriveSecondary;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static RobotContainer instance = null;

  // The robot's controllers
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;
  private final XboxController xboxTester;
  
  // Robot Subsystems
  // private final DriveSecondary closedfieldrel;
  private final AbsoluteDrive closedAbsoluteDrive;
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"/swerve/falcon"));
  AbsoluteFieldDrive closedFieldAbsoluteDrive;

  public RobotContainer() {
    // Instantiate our controllers with proper ports.
    this.xboxTester = new XboxController(1);
    this.xboxOperator = new XboxController(2);
    this.xboxDriver = new XboxController(3);

    // Controller Throttle Mappings
    // this.drive.setDefaultCommand(new FalconDrive(drive, limelight, xboxDriver));

    closedAbsoluteDrive = new AbsoluteDrive(swerveSubsystem, 
    () -> MathUtil.applyDeadband(xboxDriver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    () -> MathUtil.applyDeadband(xboxDriver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    () -> xboxDriver.getRightX(),
    () -> xboxDriver.getRightY(), false);

    closedFieldAbsoluteDrive = new AbsoluteFieldDrive(swerveSubsystem,
    () -> MathUtil.applyDeadband(xboxDriver.getLeftY(),OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(xboxDriver.getLeftX(),OperatorConstants.LEFT_X_DEADBAND),
    () -> -xboxDriver.getRightY(), false);

    System.out.println(xboxDriver.getRightX()+","+xboxDriver.getRightY());

    swerveSubsystem.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDrive : closedFieldAbsoluteDrive);

    
  }

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() { // Command

    return closedAbsoluteDrive;
    
  }

  // Reset encoders for auto
  public void resetForAuto(Pose2d pose) {
    swerveSubsystem.resetOdometry(pose);
  }

  public void prepareForAuto() {
    
  }

  // Override commands and switch to manual control

}