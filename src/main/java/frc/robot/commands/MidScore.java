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
import frc.robot.commands.WristStraight;

public class MidScore extends SequentialCommandGroup {
    public MidScore() {
        addCommands(
            new ParallelCommandGroup(
                new WristUp(Wrist.getInstance()),
                new ElevatorMid(ElevatorPIDNonProfiled.getInstance() /*ElevatorPID.getInstance()*/)),
                new ElevatorCancel(ElevatorPIDNonProfiled.getInstance()));
    }
}