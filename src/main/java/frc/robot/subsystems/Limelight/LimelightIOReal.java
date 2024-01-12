package frc.robot.subsystems.Limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightIOReal implements LimelightIO {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        inputs.tv = table.getEntry("tv").getDouble(0);
        inputs.tx = table.getEntry("tx").getDouble(0);
        inputs.ty = table.getEntry("ty").getDouble(0);
        inputs.area = table.getEntry("ta").getDouble(0);
    }
}