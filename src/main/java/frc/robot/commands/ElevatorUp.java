package frc.robot.commands;

import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ElevatorPIDNonProfiled;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FalconDrivetrain;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ElevatorUp extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorPIDNonProfiled m_elevatorSubsystem;
  // private final ElevatorPID m_elevatorSubsystem;

  private final double kHeightUpPosition = 0.715; // meters
  public ElevatorUp(ElevatorPIDNonProfiled subsystem /*ElevatorPID subsystem*/) {
    m_elevatorSubsystem = subsystem;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // follow position control to goal state
    m_elevatorSubsystem.setToVelocityControlMode(false);
    m_elevatorSubsystem.setGoal(kHeightUpPosition);
    FalconDrivetrain.getInstance().setBalancing(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_elevatorSubsystem.setToVelocityControlMode(true);
    // m_elevatorSubsystem.setVelocitySetpoint(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorSubsystem.atGoal() || m_elevatorSubsystem.m_velocityControlEnabled;
  }
}
