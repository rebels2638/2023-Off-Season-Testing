package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.lib.input.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

/** An example command that uses an example subsystem. */
public class TurretController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Turret m_turret;
  private XboxController m_controller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurretController(Turret subsystem, XboxController op) {
    m_turret = subsystem;
    m_controller = op;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double input = m_controller.getLeftX();
    // m_turret.setPercentOutput(input);
    // System.out.println("input" + input);

    m_turret.setGoal(0.5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turret.atGoal();
  }
}


