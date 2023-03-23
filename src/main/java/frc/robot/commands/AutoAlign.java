// package frc.robot.commands;

// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.CommandBase;


// public class AutoAlign extends CommandBase{
//     private double tv; 
//     private double tx;
//     private double ty;
//     private double ta;

//     public AutoAlign(){
//         tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); //Any valid targets (0,1)
//         tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); // horizontal offset (degrees)
//         ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0); //Vertical offset (degrees)
//         ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0); //target Area


//     }
//     public void initilize(){

//     }
//     public void execute(){
//         tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
//         tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
//         ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
//         ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);


//     }
//     public double angleOffset(){
//         double targetOffsetAngle_Vertical = ty;
//         double limelightMountAngleDegrees = 25.0; //The Angle from the center of the limelight lens to the floor;
//         double limelightLensHeightCentimeters = 20.0; //Distance from the center of the limelight camera lense to the floor; 
//         double goalHeightCentimeters = 60.0; //distance from the target to the floor
//         double angletoGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
//         double angleToGoalRadians = angletoGoalDegrees * (Math.PI/180);
//         double distanceFromLimelightToGoalCentimeters = (goalHeightCentimeters - limelightLensHeightCentimeters)/Math.tan(angleToGoalRadians);
//          return distanceFromLimelightToGoalCentimeters;
//     }
//     public void end(boolean interrupted){

//     }
//     public boolean isFinished(){
//         return false;
//     }
    
// }
