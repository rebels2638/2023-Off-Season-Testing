package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pivot.Intake;

public class RollIntake extends CommandBase{
    private final Intake intakeSubsystem;
    public RollIntake(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() { 
        intakeSubsystem.setVelocityRadSec(Math.toRadians(150));
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.isSpike();
    }
}
