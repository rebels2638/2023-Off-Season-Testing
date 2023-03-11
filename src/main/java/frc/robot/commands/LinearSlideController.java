package frc.robot.commands;

import frc.lib.input.XboxController;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LinearSlide;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class LinearSlideController extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final LinearSlide m_LinearSlide;
    private final XboxController m_operator;
    public boolean extend = true;

    public LinearSlideController(LinearSlide subsystem, XboxController xboxOperator) {
        m_LinearSlide = subsystem;
        m_operator = xboxOperator;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // follow position control to goal state

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // double inputPercent = m_operator.getRightX();
      // m_LinearSlide.setPercentOutput(inputPercent);
        if (m_operator.getRightBumper().getAsBoolean()) {
            System.out.println("IN!");
            m_LinearSlide.setPercentOutput(-0.5);
        } else if(m_operator.getLeftBumper().getAsBoolean()) {
            m_LinearSlide.setPercentOutput(0.5);
            System.out.println("OUT!");
        } else {
            m_LinearSlide.setPercentOutput(0);
            System.out.println("stoped!!");
        }
  }

    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

    // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
