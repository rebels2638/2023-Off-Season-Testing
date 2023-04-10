package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.NearGrid;
import frc.robot.commands.TimerCommand;
import frc.robot.commands.elevator.ElevatorUp;
import frc.robot.commands.linslide.LinSlideFullyOut;
import frc.robot.commands.wrist.WristUp;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ElevatorPIDNonProfiled;
import frc.robot.subsystems.LinSlidePiston;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.Wrist;

public class HighScore extends ParallelRaceGroup {

    public HighScore() {
        addCommands(
                new NearGrid(),
                new ParallelCommandGroup(
                        new ParallelRaceGroup(new WristUp(Wrist.getInstance()), new TimerCommand(2)),
                        new ParallelRaceGroup(new ElevatorUp(ElevatorPIDNonProfiled.getInstance()),
                                new TimerCommand(2.5)),
                        new ParallelRaceGroup(
                                new LinSlideFullyOut(LinearSlide.getInstance(), LinSlidePiston.getInstance()),
                                new TimerCommand(4))));
    }
}