package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightNT {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // Whether the limelight has any valid targets (0 or 1)
    public static boolean getTv() {
        return (table.getEntry("tv").getNumber(0)).intValue() == 1;
    }

    // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees |
    // LL2: -29.8 to 29.8 degrees)
    public static double getTx() {
        return table.getEntry("tx").getDouble(0.0);
    }

    // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees
    // | LL2: -24.85 to 24.85 degrees)
    public static double getTy() {
        return table.getEntry("ty").getDouble(0.0);
    }

    // Target Area (0% of image to 100% of image)
    public static double getTa() {
        return table.getEntry("ta").getDouble(0.0);
    }

    public static double getDistance() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 17.5;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 34;

        // distance from the target to the floor
        double goalHeightInches = 107.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        // calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
                / Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches;
    }

    /*
     * ledMode Sets limelight’s LED state
     * 0 use the LED Mode set in the current pipeline
     * 1 force off
     * 2 force blink
     * 3 force on
     */
    public static void setLimeLightLED(int mode) {
        table.getEntry("ledMode").setNumber(mode);
    }

    /*
     * camMode Sets limelight’s operation mode
     * 0 Vision processor
     * 1 Driver Camera (Increases exposure, disables vision processing)
     */
    public static void setCamMode(int mode) {
        table.getEntry("camMode").setNumber(mode);
    }

    /*
     * pipeline Sets limelight’s current pipeline
     * 0 .. 9 Select pipeline 0..9
     */
    public static void setLimeLightPipeline(int mode) {
        table.getEntry("pipeline").setNumber(mode);
    }

}
