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
import frc.robot.subsystems.Wrist;
import frc.robot.commands.ElevatorCancel;

public class ElevatorDownLinSlideIn extends ParallelCommandGroup {
    public ElevatorDownLinSlideIn() {
        addCommands(
                new SequentialCommandGroup(
                        new WristTurtle(Wrist.getInstance()),
                        new InstantCommand(() -> System.out.println("WRIST IN")),
                        new ParallelCommandGroup(
                                new LinSlideFullyIn(LinearSlide.getInstance(), LinSlidePiston.getInstance()),
                                new SequentialCommandGroup(
                                        new TimerCommand(0.5),
                                        new InstantCommand(() -> System.out.println("TIMER ENDED")),
                                        new ElevatorDown(ElevatorPIDNonProfiled.getInstance())))));
    }
}