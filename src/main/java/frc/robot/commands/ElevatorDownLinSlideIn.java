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

public class ElevatorDownLinSlideIn extends SequentialCommandGroup {
        public ElevatorDownLinSlideIn() {
                addCommands(
                                new WristTurtle(Wrist.getInstance()),
                                new InstantCommand(() -> System.out.println("WRIST IN")),
                                new ParallelCommandGroup(
                                                new LinSlideFullyIn(LinearSlide.getInstance(),
                                                                LinSlidePiston.getInstance()),
                                                new ElevatorDown(ElevatorPIDNonProfiled.getInstance())),
                                new InstantCommand(() -> System.out.println("Fsfsfsfs")));
        }
}