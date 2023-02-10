package frc.robot.commands;

import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FourBarArm;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class FourBarUp extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FourBarArm m_fourbararmSubsytem;

  private final double kHeightUpRad = 0.01; // radians
  private final TrapezoidProfile.State kGoalState = new TrapezoidProfile.State(kHeightUpRad, 0.0);

  public FourBarUp(FourBarArm subsystem) {
    m_fourbararmSubsytem = subsystem;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // follow position control to goal state
    m_fourbararmSubsytem.setToVelocityControlMode(false);
    m_fourbararmSubsytem.setGoal(kGoalState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_fourbararmSubsytem.setToVelocityControlMode(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_fourbararmSubsytem.atGoal() || m_fourbararmSubsytem.m_velocityControlEnabled;
  }
}
