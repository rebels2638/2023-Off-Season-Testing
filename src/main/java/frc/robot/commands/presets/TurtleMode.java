package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TimerCommand;
import frc.robot.commands.elevator.ElevatorDown;
import frc.robot.commands.linslide.LinSlideFullyIn;
import frc.robot.commands.wrist.WristTurtle;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ElevatorPIDNonProfiled;
import frc.robot.subsystems.LinSlidePiston;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.Wrist;

public class TurtleMode extends ParallelCommandGroup {
	public TurtleMode() {
		addCommands(
				new WristTurtle(Wrist.getInstance()),
				new LinSlideFullyIn(LinearSlide.getInstance(),
						LinSlidePiston.getInstance()),
				new ParallelRaceGroup(
						Commands.waitUntil(LinearSlide.getInstance()::sufficientlyIn).andThen(
								new ElevatorDown(ElevatorPIDNonProfiled.getInstance() /* ElevatorPID.getInstance() */)),
						new TimerCommand(2)));
	}
}