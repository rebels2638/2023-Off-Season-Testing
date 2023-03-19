package frc.robot.commands;

import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class WristDown extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Wrist m_armSubsystem;

//   private final double kHeightUpPosition = 0.381; // meters
//   private final TrapezoidProfile.State kGoalState = new TrapezoidProfile.State(kHeightUpPosition, 0.0);
  
  private final double goalAngle = -Math.PI / 4; // radians

  public WristDown(Wrist subsystem) {
    m_armSubsystem = subsystem;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // follow position control to goal state
    m_armSubsystem.setToVelocityControlMode(false);
    m_armSubsystem.setGoal(goalAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setToVelocityControlMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_armSubsystem.atGoal() || m_armSubsystem.m_velocityControlEnabled;
  }
}