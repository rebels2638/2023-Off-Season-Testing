// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.FalconDrivetrain;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// /** An example command that uses an example subsystem. */
// public class RotAlign extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final FalconDrivetrain m_Drivetrain;

//   // x1, y1, x2, y2,...
//   private final double[] wayPoints = {15.03, 1.25}; //TODO: these are garbaje! x, y, x, y
//   private final double[] scoringLocations = {0.3429572}; //angles in radians 
//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public RotAlign(FalconDrivetrain drive) {
//     m_Drivetrain = drive;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(drive);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }

// public Rotation2d generateTrajectory() {


//     Pose2d startPose = PoseEstimator.getInstance().getCurrentPose();
//     Translation2d poseFinder = new Translation2d(startPose.getRotation().getCos() * poseFinerDistance + startPose.getX(),
//        (startPose.getRotation().getSin() * poseFinerDistance + startPose.getY()) );

//     double bestX = 999999;
//     double bestY = 999999;
//     // double bestScoreX = 0;
//     // double bestScoreY = 0;
//     double scoreingAngle = 0;
//     int index = 0;

//     for (int i = 0; i < wayPoints.length; i++) {
//       // x 
//       if (i % 2 == 0) {
//         if ((Math.abs(wayPoints[i] - startPose.getX()) < Math.abs(bestX - startPose.getX())) && (Math.abs(wayPoints[i] - startPose.getY()) < Math.abs(bestY - startPose.getY()))) {
//           bestX = wayPoints[i];
//           bestY = wayPoints[i + 1];
//           scoreingAngle = scoringLocations[index];
//         }
//         index++;

//         }
//       }
    
    
//     // double endRot = startPose.getRotation().getRadians();
//     // // to the right of the robot and up
//     // if ( bestScoreX - bestX > 0 && bestScoreY - bestY > 0)  {
//     //     endRot = Math.atan( bestScoreY - bestY ) / ( bestScoreX - bestX);
//     // }
//     // // to the right of the robot and down
//     // else if ( bestScoreX - bestX > 0 && bestY - bestScoreY  > 0) {
//     //     endRot = -(Math.atan( bestY - bestScoreY ) / ( bestScoreX - bestX));
//     // }
//     // // to the left of the robot and up
//     // else if ( bestX - bestScoreX  > 0 && bestScoreY - bestY > 0) {
//     //     endRot = Math.PI - Math.atan( bestY - bestScoreY ) / ( bestX - bestScoreX );
//     // }
//     // // to the left of the robot and down
//     // else if ( bestX - bestScoreX  > 0 && bestY - bestScoreY > 0) {
//     //     endRot = Math.atan( bestY - bestScoreY ) / ( bestX - bestScoreX ) + Math.PI;
//     // }   
    
    
//     Pose2d endPose = new Pose2d(new Translation2d(bestX, bestY),
//         new Rotation2d( scoreingAngle));
      
//     ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();

//     TrajectoryConfig config = new TrajectoryConfig(.75, .25); // TODO: get the proper accels and max velocities

//     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPose, interiorWaypoints, endPose, config);

//     return trajectory;

//   }
