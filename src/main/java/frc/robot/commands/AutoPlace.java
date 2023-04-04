package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ElevatorPIDNonProfiled;
import frc.robot.subsystems.LinSlidePiston;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.ElevatorCancel;

public class AutoPlace extends SequentialCommandGroup {
    public AutoPlace() {
        addCommands(
            new ParallelRaceGroup(new ElevatorUpLinSlideoutAuto(), new TimerCommand(5)),
            new ParallelRaceGroup(new Place(), new TimerCommand(5)),
            new ParallelRaceGroup(new ElevatorDownLinSlideIn(), new TimerCommand(5))
            );
    }
}