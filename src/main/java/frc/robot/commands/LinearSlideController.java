package frc.robot.commands;

import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LinearSlide;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class LinearSlideController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LinearSlide m_LinearSlide;
  public boolean extend = true;
  

  public LinearSlideController(LinearSlide subsystem) {
    m_LinearSlide = subsystem;
    
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
    if (extend) {
      m_LinearSlide.setPercentOutput(.1);
    }
    else {
      m_LinearSlide.setPercentOutput(-.1);
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
