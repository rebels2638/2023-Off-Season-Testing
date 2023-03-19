// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FalconDrivetrain;
import frc.lib.RebelUtil;
import frc.lib.input.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class FalconDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FalconDrivetrain m_driveSubsystem;
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
  public FalconDrive(FalconDrivetrain driveSubsystem, XboxController controller) {
    xboxDriver = controller;
    m_driveSubsystem = driveSubsystem;
    rateLimiter = new SlewRateLimiter(MAX_FORWARD_ACCEL, MAX_BACKWARD_ACCEL, 0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed = RebelUtil.linearDeadband(xboxDriver.getLeftY(), 0.1) * MAX_FORWARD_SPEED * (1 - xboxDriver.getLeftTrigger());
    double turnSpeed = RebelUtil.linearDeadband(-xboxDriver.getRightX(), 0.1) * MAX_TURN_SPEED * (1 - xboxDriver.getRightTrigger());
    
    forwardSpeed = rateLimiter.calculate(forwardSpeed);

    m_driveSubsystem.drive(forwardSpeed, turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
