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
                        new WristTurtle(Wrist.getInstance()),
                        new LinSlideFullyIn(LinearSlide.getInstance(),
                                                LinSlidePiston.getInstance()),
                        new ParallelRaceGroup(new ElevatorDown(ElevatorPIDNonProfiled.getInstance() /* ElevatorPID.getInstance()*/), new TimerCommand(2)));
        }
}