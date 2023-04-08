package frc.robot.commands;

import java.time.Instant;

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

public class lowGrabAuto extends SequentialCommandGroup {
        public lowGrabAuto() {
                addCommands(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> Claw.getInstance().push()),
                                new WristDown(Wrist.getInstance())),
                                new ElevatorDown(ElevatorPIDNonProfiled.getInstance() /*ElevatorPID.getInstance()*/),
                                new InstantCommand(() -> Claw.getInstance().pull()),
                                new WristTurtle(Wrist.getInstance()));
        }
}