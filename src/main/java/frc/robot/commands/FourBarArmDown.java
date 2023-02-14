package frc.robot.commands;

import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FourBarArmPID;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/** An example command that uses an example subsystem. */
public class FourBarArmDown extends CommandBase{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FourBarArmPID m_fourBarArmSubsystem;

  private final double kHeightDownPosition = 0; // meters
  private final TrapezoidProfile.State kGoalState = new TrapezoidProfile.State(kHeightDownPosition, 0.0);

  public FourBarArmDown(FourBarArmPID subsystem) {
    
    m_fourBarArmSubsystem = subsystem;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // follow position control to goal state
    m_fourBarArmSubsystem.setToVelocityControlMode(true);
    m_fourBarArmSubsystem.setVelocitySetpoint(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_fourBarArmSubsystem.setToVelocityControlMode(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_fourBarArmSubsystem.atGoal() || m_fourBarArmSubsystem.m_velocityControlEnabled;
  }
}
