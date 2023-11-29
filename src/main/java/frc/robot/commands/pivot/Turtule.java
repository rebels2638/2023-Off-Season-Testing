package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pivot.Pivot;

public class Turtule extends CommandBase {

    private final Pivot pivotSubsystem;
    public Turtule(Pivot pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute() { 
        pivotSubsystem.setDegAngle(200);
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.reachedSetpoint();
    }
}
