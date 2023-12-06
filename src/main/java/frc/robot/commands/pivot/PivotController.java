package frc.robot.commands.pivot;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.input.XboxController;
import frc.robot.subsystems.pivot.Pivot;

public class PivotController extends CommandBase{

    Pivot pivotSubsystem = null;
    XboxController controller;

    public PivotController(Pivot pivotSubsystem, XboxController controller) {
        this.pivotSubsystem = pivotSubsystem;
        this.controller = controller;
        addRequirements(pivotSubsystem);
    }
    @Override
    public void execute() { 

        pivotSubsystem.setDegAngle(60 * controller.getLeftY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

