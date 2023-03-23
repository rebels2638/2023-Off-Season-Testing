// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.FalconDrivetrain;
// import frc.robot.subsystems.PoseEstimator;

// import java.util.ArrayList;

// import edu.wpi.first.math.Vector;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.RamseteController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;
// import frc.robot.utils.ConstantsFXDriveTrain.DriveConstants;

// /** An example command that uses an example subsystem. */
// public class AutoNotch extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final FalconDrivetrain driveTrain;
//   private RamseteCommand ramseteCommand;
//   private Pose2d startPose;
//   private Pose2d endPose;
//   RamseteController controller = new RamseteController();
//   Trajectory goalTrajectory;
//   private double startTime = Timer.getFPGATimestamp();
//   private boolean finished = false;

//   // x1, y1, x2, y2,...
//   private final double[] wayPoints = {1, 2, 1,31, 3123,312, 3,3}; //TODO: these are garbaje! x, y, x, y

//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public AutoNotch(FalconDrivetrain subsystem) {
  
//     driveTrain = subsystem;

//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(subsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//     // Using the default constructor of RamseteController. Here
//     // the gains are initialized to 2.0 and 0.7.
    

//     goalTrajectory = generateTrajectory();
    
//     // ramseteCommand =
//     //     new RamseteCommand(
//     //         trajectory,
//     //         PoseEstimator::getCurrentPose,
//     //         new RamseteController(),
//     //         driveTrain.m_feedforward,
//     //         driveTrain.m_kinematics,
//     //         driveTrain::getWheelSpeeds,
//     //         new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
//     //         new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
//     //         // RamseteCommand passes volts to the callback
//     //         driveTrain::setVoltageFromAuto,
//     //         driveTrain);

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
    
//     ChassisSpeeds adjustedSpeeds = controller.calculate(PoseEstimator.getInstance().getCurrentPose(), goalTrajectory.sample(startTime - Timer.getFPGATimestamp()));
//     DifferentialDriveWheelSpeeds wheelSpeeds = driveTrain.m_kinematics.toWheelSpeeds(adjustedSpeeds);
//     driveTrain.setSpeeds(wheelSpeeds);
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }

//   public Trajectory generateTrajectory() {


//     Pose2d startPose = PoseEstimator.getInstance().getCurrentPose();

//     double bestX = 999999;
//     double bestY = 999999;

//     for (int i = 0; i < wayPoints.length; i++) {
//       // x 
//       if (i % 2 == 0) {
//         if (Math.abs(wayPoints[i] - startPose.getX()) > Math.abs(bestX - startPose.getX())) {
//           bestX = wayPoints[i];
//         }
//       }
//       else{
//         if (Math.abs(wayPoints[i] - startPose.getY()) > Math.abs(bestY - startPose.getY())) {
//           bestY = wayPoints[i];
//         }
//       }
//     }

//     Pose2d endPose = new Pose2d(new Translation2d(bestX, bestY),
//         new Rotation2d(Math.atan(( bestY - startPose.getY() ) / ( bestX - startPose.getX()) )));
      
//     var interiorWaypoints = new ArrayList<Translation2d>();
    
//     TrajectoryConfig config = new TrajectoryConfig(1, 1); // TODO: get the proper accels and max velocetys

//     var trajectory = TrajectoryGenerator.generateTrajectory(startPose, interiorWaypoints, endPose, config);

//     return trajectory;

//   }
// }
