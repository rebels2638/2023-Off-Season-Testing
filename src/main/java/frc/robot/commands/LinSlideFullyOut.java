package frc.robot.commands;

import frc.lib.input.XboxController;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LinSlidePiston;
import frc.robot.subsystems.LinearSlide;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class LinSlideFullyOut extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final LinearSlide m_linslide;
    private final LinSlidePiston m_piston;
    public int minEncoderVelo = 1000;

    public LinSlideFullyOut(LinearSlide linslide, LinSlidePiston piston) {
        m_linslide = linslide;
        m_piston = piston;

        addRequirements(linslide, piston);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_linslide.setPercentOutput(0.0);
        m_piston.pull();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

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
