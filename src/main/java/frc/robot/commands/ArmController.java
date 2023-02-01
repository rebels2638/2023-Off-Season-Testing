// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmPIDandFeedForward;
import frc.lib.input.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArmController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
 // private final Arm m_armSubsystem;
  private final ArmPIDandFeedForward m_armSubsystem;
  private final XboxController e_controller; // e_controller is elevator's controller
  private final double kRotationMultiplier = 0.1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmController(ArmPIDandFeedForward armSubsystem, XboxController controller) {
    e_controller = controller;
    m_armSubsystem = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double inputPercent = e_controller.getRightY();
    
    double currentAngle = m_armSubsystem.getRadRotation();
    m_armSubsystem.setAngle(currentAngle + inputPercent * kRotationMultiplier);
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
