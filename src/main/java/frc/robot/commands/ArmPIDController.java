// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ArmPID;
import frc.lib.RebelUtil;
import frc.lib.input.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;

/** An example command that uses an example subsystem. */
public class ArmPIDController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmPID m_armPID;
  private final XboxController e_controller; // e_controller is elevator's controller

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmPIDController(ArmPID armPIDSubsystem, XboxController controller) {
    e_controller = controller;
    m_armPID = armPIDSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armPID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armPID.setToVelocityControlMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double desiredVelo = e_controller.getRightY() * ArmPID.kMaxSpeed;
    m_armPID.setVelocitySetpoint(desiredVelo);
    m_armPID.setToVelocityControlMode(true);
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