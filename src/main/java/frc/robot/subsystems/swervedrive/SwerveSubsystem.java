package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import frc.lib.swervelib.SwerveController;
import frc.lib.swervelib.SwerveDrive;
import frc.lib.swervelib.math.SwerveKinematics2;
import frc.lib.swervelib.math.SwerveModuleState2;
import frc.lib.swervelib.parser.SwerveControllerConfiguration;
import frc.lib.swervelib.parser.SwerveDriveConfiguration;
import frc.lib.swervelib.parser.SwerveParser;
import frc.lib.swervelib.telemetry.SwerveDriveTelemetry;
import frc.lib.swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
//import frc.robot.subsystems.aprilTagVision.AprilTagVision;


public class SwerveSubsystem extends SubsystemBase
{  
  /**
   * Swerve drive object.
   */
  private final SwerveDrive swerveDrive;
  
  //AprilTagVision aprilTagVision;

  /**
   * The auto builder for PathPlanner, there can only ever be one created so we save it just incase we generate multiple
   * paths with events.
   */
  private SwerveAutoBuilder autoBuilder = null;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  //private static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0.16026, 0.0023745, 2.774E-05);

  // public SwerveSubsystem(File directory, AprilTagVision aprilTagVision)
  public SwerveSubsystem(File directory)
  {
    //this.aprilTagVision = aprilTagVision;
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive();
    } catch (Exception e)
    {
      throw new IllegalArgumentException("File not found.");
    }

    swerveDrive.setMotorIdleMode(true);
    
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg);
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   * @param isOpenLoop    Whether to use closed-loop velocity control.  Set to true to disable closed-loop.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
  {
    swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
  }

  @Override
  public void periodic()
  {
     swerveDrive.updateOdometry();

    // Pose2d currentPose2d = swerveDrive.getPose();
    // Pose3d refrencePose = new Pose3d( new Translation3d( currentPose2d.getX(), currentPose2d.getY(), 0), 
    //   new Rotation3d(currentPose2d.getRotation().getRadians(), 0, 0));
      
    // Pose3d estimatedPose = aprilTagVision.getEstimatedPosition(refrencePose);

    // if (estimatedPose != null) {
    //   swerveDrive.addVisionMeasurement(estimatedPose.toPose2d(),
    //     Timer.getFPGATimestamp() - aprilTagVision.getPiplineLatency(),
    //     true, 1);
    // }
    //log all tlemetry to a log file
    Logger.getInstance().recordOutput("swerve/moduleCount", SwerveDriveTelemetry.moduleCount);
    Logger.getInstance().recordOutput("swerve/wheelLocations", SwerveDriveTelemetry.wheelLocations);
    Logger.getInstance().recordOutput("swerve/measuredStates", SwerveDriveTelemetry.measuredStates);
    Logger.getInstance().recordOutput("swerve/desiredStates", SwerveDriveTelemetry.desiredStates);
    Logger.getInstance().recordOutput("swerve/robotRotation", SwerveDriveTelemetry.robotRotation);
    Logger.getInstance().recordOutput("swerve/maxSpeed", SwerveDriveTelemetry.maxSpeed);
    Logger.getInstance().recordOutput("swerve/rotationUnit", SwerveDriveTelemetry.rotationUnit);
    Logger.getInstance().recordOutput("swerve/sizeLeftRight", SwerveDriveTelemetry.sizeLeftRight);
    Logger.getInstance().recordOutput("swerve/sizeFrontBack", SwerveDriveTelemetry.sizeFrontBack);
    Logger.getInstance().recordOutput("swerve/forwardDirection", SwerveDriveTelemetry.forwardDirection);
    Logger.getInstance().recordOutput("swerve/maxAngularVelocity", SwerveDriveTelemetry.maxAngularVelocity);
    Logger.getInstance().recordOutput("swerve/measuredChassisSpeeds", SwerveDriveTelemetry.measuredChassisSpeeds);
    Logger.getInstance().recordOutput("swerve/desiredChassisSpeeds", SwerveDriveTelemetry.desiredChassisSpeeds);
  
  }

  @Override
  public void simulationPeriodic()
  {
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveKinematics2} of the swerve drive.
   */
  public SwerveKinematics2 getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return swerveDrive.getYaw();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians());
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians());
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
    
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp(), true, 4);
  }

  /**
   * Factory to fetch the PathPlanner command to follow the defined path.
   *
   * @param path             Path planner path to specify.
   * @param constraints      {@link PathConstraints} for {@link com.pathplanner.lib.PathPlanner#loadPathGroup} function
   *                         limiting velocity and acceleration.
   * @param eventMap         {@link java.util.HashMap} of commands corresponding to path planner events given as
   *                         strings.
   * @param translation      The {@link PIDConstants} for the translation of the robot while following the path.
   * @param rotation         The {@link PIDConstants} for the rotation of the robot while following the path.
   * @param useAllianceColor Automatically transform the path based on alliance color.
   * @return PathPlanner command to follow the given path.
   */
  public Command creatPathPlannerCommand(String path, PathConstraints constraints, Map<String, Command> eventMap,
                                         PIDConstants translation, PIDConstants rotation, boolean useAllianceColor)
  {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(path, constraints);
//    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
//      Pose2d supplier,
//      Pose2d consumer- used to reset odometry at the beginning of auto,
//      PID constants to correct for translation error (used to create the X and Y PID controllers),
//      PID constants to correct for rotation error (used to create the rotation controller),
//      Module states consumer used to output to the drive subsystem,
//      Should the path be automatically mirrored depending on alliance color. Optional- defaults to true
//   )
    if (autoBuilder == null)
    {
      autoBuilder = new SwerveAutoBuilder(
          swerveDrive::getPose,
          swerveDrive::resetOdometry,
          translation,
          rotation,
          swerveDrive::setChassisSpeeds,
          eventMap,
          useAllianceColor,
          this
      );
    }

    return autoBuilder.fullAuto(pathGroup);
  }

   /**
   * Factory to fetch the PathPlanner command to follow the defined path.
   *
   * @param trajectory        Path planner trajectory to specify.
   * @param eventMap         {@link java.util.HashMap} of commands corresponding to path planner events given as
   *                         strings.
   * @param translation      The {@link PIDConstants} for the translation of the robot while following the path.
   * @param rotation         The {@link PIDConstants} for the rotation of the robot while following the path.
   * @param useAllianceColor Automatically transform the path based on alliance color.
   * @return PathPlanner command to follow the given path.
   */
  public Command creatPathPlannerCommand( PathPlannerTrajectory trajectory, Map<String, Command> eventMap, 
                                          PIDConstants translation, PIDConstants rotation, boolean useAllianceColor)
  {
//    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
//      Pose2d supplier,
//      Pose2d consumer- used to reset odometry at the beginning of auto,
//      PID constants to correct for translation error (used to create the X and Y PID controllers),
//      PID constants to correct for rotation error (used to create the rotation controller),
//      Module states consumer used to output to the drive subsystem,
//      Should the path be automatically mirrored depending on alliance color. Optional- defaults to true
//   )
    if (autoBuilder == null)
    {
      autoBuilder = new SwerveAutoBuilder(
          swerveDrive::getPose,
          swerveDrive::resetOdometry,
          translation,
          rotation,
          swerveDrive::setChassisSpeeds,
          eventMap,
          useAllianceColor,
          this
      );
    }

    return autoBuilder.fullAuto(trajectory);
  }
  /**
   * Set the module states (azimuth and velocity) directly. Used primarily for auto paths.
   *
   * @param desiredStates A list of SwerveModuleStates to send to the modules.
   * @param isOpenLoop    Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   */
  public void setModuleStates(SwerveModuleState2[] desiredStates, boolean isOpenLoop)
  {
    swerveDrive.setModuleStates(desiredStates, isOpenLoop);
  }

  /**
   * Gets the current module states (azimuth and velocity)
   *
   * @return A list of SwerveModuleStates containing the current module states
   */
  public SwerveModuleState2[] getStates()
  {
    return swerveDrive.getStates();
  }

  public void resetGyro() {
    swerveDrive.setGyro(new Rotation3d());
  }

}