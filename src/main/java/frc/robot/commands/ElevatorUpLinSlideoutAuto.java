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

public class ElevatorUpLinSlideoutAuto extends SequentialCommandGroup {

    public ElevatorUpLinSlideoutAuto() {
        addCommands(
                new ParallelCommandGroup(
                    new ParallelRaceGroup(new WristUp(Wrist.getInstance()), new TimerCommand(2)),
                    new ElevatorUp(ElevatorPIDNonProfiled.getInstance() /*ElevatorPID.getInstance()*/)),
                new ParallelRaceGroup(new LinSlideFullyOut(LinearSlide.getInstance(), LinSlidePiston.getInstance()),new TimerCommand(4)),
                new InstantCommand(() -> System.out.println("The wrist is working")));
    }
}