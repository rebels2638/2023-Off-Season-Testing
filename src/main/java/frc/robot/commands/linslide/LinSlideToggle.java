package frc.robot.commands.linslide;

import frc.lib.input.XboxController;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LinSlidePiston;
import frc.robot.subsystems.LinearSlide;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** An example command that uses an example subsystem. */
public class LinSlideToggle extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final LinearSlide m_LinearSlide;
    private final LinSlidePiston m_linPiston;
    private XboxController m_operator;
    public boolean extend = true;
    public static boolean state = false;

    public LinSlideToggle(LinearSlide subsystem, LinSlidePiston piston, XboxController xboxOperator) {
        m_LinearSlide = subsystem;
        m_linPiston = piston;
        m_operator = xboxOperator;

        addRequirements(subsystem);
    }

    public LinSlideToggle(LinearSlide subsystem, LinSlidePiston piston) {
      m_LinearSlide = subsystem;
      m_linPiston = piston;

      addRequirements(subsystem);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // follow position control to goal state
        state = m_LinearSlide.getCurrentEncoderPosition() > 20000;
        if (state) {
          var in = new LinSlideFullyIn(m_LinearSlide, m_linPiston);
          in.schedule();
          System.out.println("in: " + in);
      } else {
          var out = new LinSlideFullyOut(m_LinearSlide, m_linPiston);
          out.schedule();
          System.out.println("out: " + out);
      }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // double inputPercent = m_operator.getRightX();
      // m_LinearSlide.setPercentOutput(inputPercent);

      // m_LinearSlide.setPercentOutput(m_operator.getRightX());

      // if (m_operator.getRightBumper().getAsBoolean()) {
      //     // System.out.println("IN!");
      //     m_LinearSlide.setPercentOutput(-0.2);
      // } else if(m_operator.getLeftBumper().getAsBoolean()) {
      //     m_LinearSlide.setPercentOutput(0.2);
      //     // System.out.println("OUT!");
      // } else {
      //     m_LinearSlide.setPercentOutput(0);
      //     // System.out.println("stoped!!");
      // }
  }

    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

    // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
