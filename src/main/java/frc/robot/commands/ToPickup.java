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

public class ToPickup extends ParallelCommandGroup {
        public ToPickup() {
                addCommands(
                                new SequentialCommandGroup(
                                                new WristTurtle(Wrist.getInstance()),
                                                new TimerCommand(0.25),
                                                new WristDown(Wrist.getInstance())),
                                new ParallelCommandGroup(
                                                new LinSlideFullyIn(LinearSlide.getInstance(),
                                                                LinSlidePiston.getInstance()),
                                                new SequentialCommandGroup(
                                                        new TimerCommand(0.5),
                                                        new ParallelCommandGroup(new InstantCommand(() -> Claw.getInstance().push()),
                                                                new ElevatorDown(ElevatorPIDNonProfiled.getInstance() /*ElevatorPID.getInstance()*/)))));
        }
}