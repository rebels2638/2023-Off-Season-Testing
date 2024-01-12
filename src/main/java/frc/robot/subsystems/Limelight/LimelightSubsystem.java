package frc.robot.subsystems.Limelight;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase{
    private LimelightIO io;
    private final LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();
    public LimelightSubsystem(LimelightIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("LimelightSubsystem", inputs);
    }

    public double getAngleToTarget() {
        return inputs.tx;
    }

    public boolean hasTargets() {
        return (inputs.tv == (double)1);
    }
    
}
