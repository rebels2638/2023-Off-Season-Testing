package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pivot.Pivot;

public class PivotToCube extends CommandBase{

    private final Pivot pivotSubsystem;
    public PivotToCube(Pivot pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute() { 
        pivotSubsystem.setDegAngle(50);
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.reachedSetpoint();
    }
}
