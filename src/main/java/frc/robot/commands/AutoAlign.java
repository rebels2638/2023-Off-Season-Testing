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
        double limelightLensHeightCentimeters = 20.0;

        // distance from the target to the floor
        double goalHeightCentimeters = 60.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalCentimeters = (goalHeightCentimeters - limelightLensHeightCentimeters)/Math.tan(angleToGoalRadians);
        
        return distanceFromLimelightToGoalCentimeters;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }


    /* Trajectory planning 
     * 
     * 1. make simple networktables interface with all the cone / cube placement buttons 
     * 2. find current pose, and the pose we want to go (1 ft away from the final pose (striaght on)) and create trajectory w/ ramsette controller stuff
     *  a. assign which notches for which poles
     * 3. if statement so autorun can only happen after going past charge station pose (x coord)
     * 4. go to pose (1ft away) 
     * 5. find new pose to go in (straight 1ft)
     * 6. use limelight reflective tape to align with the pole (if we can use multiple pipelines) (or even gyro or pose) 
     * 7. align within threshold of degrees (5 or smth) and then do place preset (current + a piston drop after)
     * 
     */
}
