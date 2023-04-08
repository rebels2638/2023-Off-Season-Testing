package frc.robot.commands.presets;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorDown;
import frc.robot.commands.wrist.WristDown;
import frc.robot.commands.wrist.WristTurtle;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ElevatorPIDNonProfiled;
import frc.robot.subsystems.LinSlidePiston;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.Wrist;

public class LowGrab extends SequentialCommandGroup {
        public LowGrab() {
                addCommands(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> Claw.getInstance().push()),
                                new WristDown(Wrist.getInstance())),
                                new ElevatorDown(ElevatorPIDNonProfiled.getInstance() /*ElevatorPID.getInstance()*/),
                                new InstantCommand(() -> Claw.getInstance().pull()),
                                new WristTurtle(Wrist.getInstance()));
        }
}