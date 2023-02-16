// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.controller.RamseteController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.FalconDrivetrain;
// import frc.robot.utils.AutoConstants;
// import frc.robot.utils.ConstantsFXDriveTrain.DriveConstants;

// import java.util.Arrays;
// import java.util.function.Consumer;

// import com.pathplanner.lib.*;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.RamseteAutoBuilder;

// public final class Auto
//     extends CommandBase {
//   private FalconDrivetrain m_drive;
//   private boolean finished = false;
//   private ShuffleboardTab tab = Shuffleboard.getTab("Auto");
//   private PathPlannerTrajectory m_path;
//   private RamseteAutoBuilder m_autoBuilder;

//   public Auto(FalconDrivetrain drive) {
//     m_drive = drive;
//     m_autoBuilder = new RamseteAutoBuilder(
//       drive::getPose,
//       pose -> drive.resetOdometry(pose),
//       new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
//       drive.m_kinematics,
//       drive.m_feedforward,
//       () -> drive.getWheelSpeeds(),
//       new PIDConstants(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
//       (leftVoltage, rightVoltage) -> drive.setVoltageFromAuto(leftVoltage, rightVoltage),
//       AutoConstants.AUTO_EVENT_MAP,
//       true,
//       m_drive
//     );

//     addRequirements(drive);
//   }

//   public void setPath(final String fileName, PathConstraints constraints) {
//     m_path = PathPlanner.loadPath(fileName, constraints);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_autoBuilder.fullAuto(m_path);
//   }

//   @Override
//   public boolean isFinished() {
//     return finished;
//   }
// }
