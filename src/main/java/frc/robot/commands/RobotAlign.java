// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.DrivetrainFalcon;
// import frc.robot.subsystems.FalconDrivetrain;
// import frc.robot.subsystems.PoseEstimator;
// import frc.robot.subsystems.Turret;
// import frc.lib.RebelUtil;
// import frc.lib.input.XboxController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.CommandBase;



// /** An example command that uses an example subsystem. */
// public class RobotAlign extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final FalconDrivetrain m_driveSubsystem;
//   private final PoseEstimator poseEstimator;
//   private final Pose2d target;

//   // furthest allowed x pos on field that the robot can turn at.
  
//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public RobotAlign(FalconDrivetrain driveSubsystem, PoseEstimator poseEstimatorSubsystem, Pose2d target) {
//     m_driveSubsystem = driveSubsystem;
//     poseEstimator = poseEstimatorSubsystem;
//     this.target = target;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(driveSubsystem);
//     addRequirements(poseEstimatorSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     //m_turretSubsystem.m_velocityControlEnabled = true;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_driveSubsystem.drive(0, 0);
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }