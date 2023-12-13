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
    //     pivotSubsystem.setVelocityControlMode(true);
    //    pivotSubsystem.setVelocitySetPoint(0.1 * controller.getRightY());
       pivotSubsystem.setVoltage(3 * controller.getRightY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted){
        System.out.println("CALLED END");
        pivotSubsystem.setVelocitySetPoint(0);
        return;
    }
}

