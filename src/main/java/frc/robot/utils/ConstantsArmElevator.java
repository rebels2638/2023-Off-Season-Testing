package frc.robot.utils;

public final class ConstantsArmElevator {
    public static final double coneHeight = 0.35;
    public static final double midY = 0.87 + coneHeight;
    public static final double midX = 0;
    
    // All in meters
    public static final class ElevatorConstants {
        public static final double groundToElevatorNeutral = 0;
        public static final double groundSetpoint = 0;
        public static final double loadingStation = 0; // just the height of the loading station
        public static final double drive = 0;
        public static final double midScore = midY - ArmConstants.armLength * Math.sin(ArmConstants.midScore) - groundToElevatorNeutral;
    }

    public static final class ArmConstants {
        public static final double armLength = 0; // arm length
        public static final double groundSetpoint = 0; // radians
        public static final double loadingStation = 0; // radians
        public static final double drive = Math.PI / 4;
        public static final double midScore = Math.acos(midX / armLength); 
    }
}
