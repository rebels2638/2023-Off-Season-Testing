// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Wrist;
import frc.lib.RebelUtil;
import frc.lib.input.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;

/** An example command that uses an example subsystem. */
public class WristController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Wrist m_wrist;
  private final XboxController e_controller; // e_controller is elevator's controller
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WristController(Wrist wristSubsystem, XboxController controller) {
    e_controller = controller;
    m_wrist = wristSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_wrist.setToVelocityControlMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(m_wrist.getCurrentEncoderPosition() <= kUpperLimit && m_wrist.getCurrentEncoderPosition() >= kLowerLimit){
    double desiredVelo = e_controller.getRightY() * Wrist.kMaxSpeed;
    if(desiredVelo != 0) 
    {
      m_wrist.setToVelocityControlMode(true);
    }
      m_wrist.setVelocitySetpoint(desiredVelo);
    // m_wrist.setToVelocityControlMode(true);
    //}
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