// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ClawController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Claw m_clawSubsystem;
  private final XboxController c_controller; // e_controller is claw's controller
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClawController(Claw clawSubsystem, XboxController controller) {
    c_controller = controller;
    m_clawSubsystem = clawSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (c_controller.getAButtonPressed()) {
      m_clawSubsystem.pull();
    }
    
    else if (c_controller.getBButtonPressed()) {
      m_clawSubsystem.push();
    }
    
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
