// Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FalconDrivetrain;
import frc.robot.subsystems.GamePieceIdentifier;
import frc.robot.subsystems.PoseEstimator;

import java.util.ArrayList;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.utils.ConstantsFXDriveTrain.DriveConstants;
import java.util.HashMap;

/** An example command that uses an example subsystem. */
public class GamePiecePickup extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FalconDrivetrain driveTrain;
  private RamseteCommand ramseteCommand;
  RamseteController controller = new RamseteController();
  Trajectory goalTrajectory;
  private double startTime = Timer.getFPGATimestamp();
  private boolean finished = false;
  private PoseEstimator poseEstimator = PoseEstimator.getInstance();
  private GamePieceIdentifier gamePieceIdentifier;

  private final double clawDistanceOffset = .5;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GamePiecePickup(FalconDrivetrain driveTrain, GamePieceIdentifier gamePieceIdentifier) {
  
    this.driveTrain = driveTrain;
    this.gamePieceIdentifier = gamePieceIdentifier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain,gamePieceIdentifier);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Using the default constructor of RamseteController. Here
    // the gains are initialized to 2.0 and 0.7.
    
    if (!gamePieceIdentifier.GamePieceVisible()) {
      finished = true;
    }
    Trajectory goalTrajectory = generateTrajectory();
    
    // ramseteCommand =
    //     new RamseteCommand(
    //         goalTrajectory,
    //         PoseEstimator::getCurrentPose,
    //         new RamseteController(),
    //         driveTrain.m_feedforward,
    //         driveTrain.m_kinematics,
    //         driveTrain::getWheelSpeeds,
    //         new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
    //         new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
    //         // RamseteCommand passes volts to the callback
    //         driveTrain::setVoltageFromAuto,
    //         driveTrain);
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds adjustedSpeeds = controller.calculate(PoseEstimator.getInstance().getCurrentPose(), goalTrajectory.sample(startTime - Timer.getFPGATimestamp()));
    DifferentialDriveWheelSpeeds wheelSpeeds = driveTrain.m_kinematics.toWheelSpeeds(adjustedSpeeds);
    driveTrain.setSpeeds(wheelSpeeds);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }

  public Trajectory generateTrajectory() {

    Pose2d startPose = poseEstimator.getCurrentPose();
    Translation2d absoluteGamePieceTranslation = gamePieceIdentifier.GetAbsoluteGamePieceTranslation();
    double endRot = gamePieceIdentifier.GetAngleToRobot() + poseEstimator.getCurrentPose().getRotation().getRadians();
    double endX = absoluteGamePieceTranslation.getX() - (Math.cos(gamePieceIdentifier.GetAngleToRobot()) * clawDistanceOffset);
    double endY = absoluteGamePieceTranslation.getX() - (Math.sin(gamePieceIdentifier.GetAngleToRobot()) * clawDistanceOffset);

    Pose2d endPose = new Pose2d(endX, endY, new Rotation2d(endRot));
      
    ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();

    TrajectoryConfig config = new TrajectoryConfig(.75, .25); // TODO: get the proper accels and max velocities

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPose, interiorWaypoints, endPose, config);

    return trajectory;

  }
}
