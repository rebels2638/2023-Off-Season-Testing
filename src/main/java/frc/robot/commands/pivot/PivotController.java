package frc.robot.commands.pivot;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pivot.Pivot;

public class PivotController extends CommandBase{

    Pivot pivotSubsystem = null;
    DoubleSupplier controllerVal;

    public PivotController(Pivot pivotSubsystem, DoubleSupplier controller) {
        this.pivotSubsystem = pivotSubsystem;
        this.controllerVal = controller;
        addRequirements(pivotSubsystem);
    }
    @Override
    public void execute() { 

        pivotSubsystem.driveVoltage(Math.pow());
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.reachedSetpoint();
    }
}

}
