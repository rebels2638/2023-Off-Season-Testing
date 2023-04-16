// Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FalconDrivetrain;
import frc.robot.subsystems.PoseEstimator;

import java.util.ArrayList;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.utils.AutoConstants;
import frc.robot.utils.ConstantsFXDriveTrain.DriveConstants;
import java.util.HashMap;
import java.util.Map;

/** An example command that uses an example subsystem. */
public class AutoNotch extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FalconDrivetrain driveTrain;
  private RamseteCommand ramseteCommand;
  RamseteController controller = new RamseteController();
  private Trajectory goalTrajectory;
  private double startTime = Timer.getFPGATimestamp();
  private boolean finished = false;

  private final  double poseFinerDistance = 0.7;

  // x1, y1, x2, y2,...
  private final double[] wayPoints = {15.03, 1.25}; //TODO: these are garbaje! x, y, x, y
  private final double[] scoringLocations = {0.3429572}; //angles in radians 

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoNotch(FalconDrivetrain subsystem) {
    driveTrain = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Using the default constructor of RamseteController. Here
    // the gains are initialized to 2.0 and 0.7.

    goalTrajectory = generateTrajectory();
    startTime = Timer.getFPGATimestamp();
    
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
    ChassisSpeeds adjustedSpeeds = controller.calculate(PoseEstimator.getInstance().getCurrentPose(), goalTrajectory.sample(Timer.getFPGATimestamp() - startTime));
    DifferentialDriveWheelSpeeds wheelSpeeds = driveTrain.m_kinematics.toWheelSpeeds(adjustedSpeeds);
    // System.out.println(wheelSpeeds.leftMetersPerSecond + " " + wheelSpeeds.rightMetersPerSecond);
    driveTrain.setSpeeds(wheelSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double distPoses(Pose2d p1, Pose2d p2) {
    return p1.getTranslation().getDistance(p2.getTranslation());
  }

  public Trajectory generateTrajectory() {
    Pose2d startPose = PoseEstimator.getInstance().getCurrentPose();
    Translation2d poseFinder = new Translation2d(startPose.getRotation().getCos() * poseFinerDistance + startPose.getX(),
       (startPose.getRotation().getSin() * poseFinerDistance + startPose.getY()) );

    Pose2d bestPose = new Pose2d(999999, 999999, new Rotation2d(0.0, 0.0));
    int index = 0;

    for(Map.Entry<Integer, Pose3d> pose : AutoConstants.PoseMap.targetPoses.entrySet()) {
      Pose2d refPose = pose.getValue().toPose2d();
      if(DriverStation.getAlliance() == DriverStation.Alliance.Red) refPose = new Pose2d(refPose.getX(), AutoConstants.FIELD_WIDTH - refPose.getY(), refPose.getRotation().plus(new Rotation2d(Math.PI)));
      if(distPoses(startPose, refPose) < distPoses(startPose, bestPose)) {
        bestPose = refPose;
      }
    }
    
    
    // double endRot = startPose.getRotation().getRadians();
    // // to the right of the robot and up
    // if ( bestScoreX - bestX > 0 && bestScoreY - bestY > 0)  {
    //     endRot = Math.atan( bestScoreY - bestY ) / ( bestScoreX - bestX);
    // }
    // // to the right of the robot and down
    // else if ( bestScoreX - bestX > 0 && bestY - bestScoreY  > 0) {
    //     endRot = -(Math.atan( bestY - bestScoreY ) / ( bestScoreX - bestX));
    // }
    // // to the left of the robot and up
    // else if ( bestX - bestScoreX  > 0 && bestScoreY - bestY > 0) {
    //     endRot = Math.PI - Math.atan( bestY - bestScoreY ) / ( bestX - bestScoreX );
    // }
    // // to the left of the robot and down
    // else if ( bestX - bestScoreX  > 0 && bestY - bestScoreY > 0) {
    //     endRot = Math.atan( bestY - bestScoreY ) / ( bestX - bestScoreX ) + Math.PI;
    // }
      
    ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();

    TrajectoryConfig config = new TrajectoryConfig(3, 2); // TODO: get the proper accels and max velocities
    config.setStartVelocity(driveTrain.m_kinematics.toChassisSpeeds(driveTrain.getWheelSpeeds()).vxMetersPerSecond);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPose, interiorWaypoints, bestPose, config);

    return trajectory;

  }
    }
