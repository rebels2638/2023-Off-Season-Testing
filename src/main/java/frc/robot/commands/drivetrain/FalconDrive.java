// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FalconDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.AutoConstants.LimelightConstants;
import frc.lib.RebelUtil;
import frc.lib.input.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class FalconDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FalconDrivetrain m_driveSubsystem;
  private final Limelight m_limelight;
  private final XboxController xboxDriver;
  private final double MAX_FORWARD_SPEED = 5;
  private final double MAX_TURN_SPEED = 5;

  private final SlewRateLimiter rateLimiter;
  private double MAX_FORWARD_ACCEL = 7;
  private double MAX_BACKWARD_ACCEL = -7;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FalconDrive(FalconDrivetrain driveSubsystem, Limelight limelight, XboxController controller) {
    xboxDriver = controller;
    m_driveSubsystem = driveSubsystem;
    m_limelight = limelight;
    rateLimiter = new SlewRateLimiter(MAX_FORWARD_ACCEL, MAX_BACKWARD_ACCEL, 0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rateLimiter.reset(m_driveSubsystem.m_kinematics.toChassisSpeeds(m_driveSubsystem.getWheelSpeeds()).vxMetersPerSecond);
    m_limelight.setMode(LimelightConstants.DRIVER_PIPELINE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed = RebelUtil.linearDeadband(xboxDriver.getLeftY(), 0.1) * MAX_FORWARD_SPEED * (1.0 - xboxDriver.getLeftTrigger() / 2.0);
    double turnSpeed = RebelUtil.linearDeadband(-xboxDriver.getRightX(), 0.1) * MAX_TURN_SPEED * (1.0 - xboxDriver.getRightTrigger() / 2.0);
    
    forwardSpeed = rateLimiter.calculate(forwardSpeed);

    m_driveSubsystem.drive(forwardSpeed, turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limelight.setMode(LimelightConstants.DEFAULT_PIPELINE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
