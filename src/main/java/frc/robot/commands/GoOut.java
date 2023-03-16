package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ElevatorPIDNonProfiled;
import frc.robot.subsystems.LinSlidePiston;
import frc.robot.subsystems.LinearSlide;
import frc.robot.commands.ElevatorCancel;

public class GoOut extends ParallelCommandGroup {
    public GoOut() {
        addCommands(
            new ElevatorUp(ElevatorPIDNonProfiled.getInstance()),
            new SequentialCommandGroup(
                new TimerCommand(1),
                new LinSlideFullyOut(LinearSlide.getInstance(), LinSlidePiston.getInstance()),
                new ElevatorCancel(ElevatorPID.getInstance())
            )
        );
    }
}