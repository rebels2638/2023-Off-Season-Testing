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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class FieldOrientedDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_driveSubsystem;
  private final XboxController xboxDriver;
  private final double MAX_FORWARD_SPEED = 1;
  private final double MAX_TURN_SPEED = 5;
  private final double MAX_TURN_ACCELERATION = 20;
  private final double CONTROL_STRENGTH = 3; // THIS MUST BE AN ODD POSITIVE INTEGER

  private final ProfiledPIDController pid = new ProfiledPIDController(2, 0, 1.5, new TrapezoidProfile.Constraints(MAX_TURN_SPEED, MAX_TURN_ACCELERATION));
  
  private Rotation2d m_headingSetpoint;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FieldOrientedDrive(Drivetrain driveSubsystem, XboxController controller) {
    xboxDriver = controller;
    m_driveSubsystem = driveSubsystem;
    pid.enableContinuousInput(-Math.PI, Math.PI);
    m_headingSetpoint = new Rotation2d();
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
    Rotation2d gyroHeading = m_driveSubsystem.getRotation2d();
    SmartDashboard.putNumber("GYRO", gyroHeading.getRadians());

    // Do not update heading if xbox controller shows close to no input
    double magnitude = new Translation2d(xboxDriver.getLeftX(), xboxDriver.getLeftY()).getNorm();
    if(magnitude > 0.1) m_headingSetpoint = headingSetpoint;
    if(magnitude < 0.15) magnitude = 0;
    SmartDashboard.putNumber("GYRO SETPOINT", m_headingSetpoint.getRadians());

    // Keep heading setpoint in the direction of minimal movement unless otherwise specified by the driver
    Rotation2d headingError = gyroHeading.minus(m_headingSetpoint);
    if(headingError.getCos() < 0.0 && !xboxDriver.getLeftBumper().getAsBoolean()) m_headingSetpoint = m_headingSetpoint.plus(new Rotation2d(Math.PI));

    // Forward speed is reduced when robot is not close to parallel with the heading setpoint (dot product)
    double forwardSpeed = Math.pow(headingError.getCos(), CONTROL_STRENGTH) * magnitude * MAX_FORWARD_SPEED;
    double turnSpeed = pid.calculate(gyroHeading.getRadians(), m_headingSetpoint.getRadians());

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
