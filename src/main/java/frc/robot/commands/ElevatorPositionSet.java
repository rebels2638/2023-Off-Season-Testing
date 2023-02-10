package frc.robot.commands;

import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ElevatorPositionSet extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorPID m_elevatorSubsystem;

  private final TrapezoidProfile.State kGoalState;

  public ElevatorPositionSet(ElevatorPID subsystem, double height) {
    m_elevatorSubsystem = subsystem;
    kGoalState = new TrapezoidProfile.State(height, 0.0);
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // follow position control to goal state
    m_elevatorSubsystem.setToVelocityControlMode(false);
    m_elevatorSubsystem.setGoal(kGoalState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.setToVelocityControlMode(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorSubsystem.atGoal() || m_elevatorSubsystem.m_velocityControlEnabled;
  }
}
