package frc.robot.commands;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoAlign extends CommandBase {
    private double tv; 
    private double tx;
    private double ty;
    private double ta;

    public AutoAlign() {
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); // any valid targets (0, 1)
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); // horizontal offset (degrees)
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0); // vertical offset (degrees)
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0); // target area

    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); // any valid targets (0, 1)
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); // horizontal offset (degrees)
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0); // vertical offset (degrees)
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0); // target area
    }

    public double angleOffset(){
        return tx;
    }

    public double distanceInches() {
        double targetOffsetAngle_Vertical = ty;

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0;

        // distance from the target to the floor
        double goalHeightInches = 60.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
        
        return distanceFromLimelightToGoalInches;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
