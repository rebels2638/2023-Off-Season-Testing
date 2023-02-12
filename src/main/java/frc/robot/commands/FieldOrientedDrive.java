// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FalconDrivetrain;
import frc.lib.RebelUtil;
import frc.lib.input.XboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class FieldOrientedDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FalconDrivetrain m_driveSubsystem;
  private final XboxController xboxDriver;
  private final double MAX_FORWARD_SPEED = 3;
  private final double MAX_TURN_SPEED = 10;
  private final double CONTROL_STRENGTH = 5; // THIS MUST BE AN ODD POSITIVE INTEGER

  private final PIDController pid = new PIDController(1, 0, 0);
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FieldOrientedDrive(FalconDrivetrain driveSubsystem, XboxController controller) {
    xboxDriver = controller;
    m_driveSubsystem = driveSubsystem;
    pid.enableContinuousInput(-Math.PI, Math.PI);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d headingSetpoint = new Rotation2d(xboxDriver.getLeftX(), xboxDriver.getLeftY());
    Rotation2d gyroHeading = m_driveSubsystem.getPose().getRotation();

    double magnitude = new Translation2d(xboxDriver.getLeftX(), xboxDriver.getLeftY()).getNorm();
    double forwardSpeed = Math.pow(gyroHeading.minus(headingSetpoint).getCos(), CONTROL_STRENGTH) * magnitude * MAX_FORWARD_SPEED;
    double turnSpeed = RebelUtil.constrain(pid.calculate(gyroHeading.getRadians(), headingSetpoint.getRadians()), -MAX_TURN_SPEED, MAX_TURN_SPEED);
    
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
