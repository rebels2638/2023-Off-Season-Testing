// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.input.XboxController;
import frc.robot.commands.ArmController;
import frc.robot.commands.ClawController;
import frc.robot.commands.Drive;
import frc.robot.commands.ElevatorCancel;
import frc.robot.commands.ElevatorController;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorPIDController;
import frc.robot.commands.ElevatorUp;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.Arm;
import frc.robot.commands.FourBarArmController;
import frc.robot.subsystems.FourBarArm;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
import frc.robot.utils.ConstantsSRXDriveTrain.DriveConstants;
public class RobotContainer {
  // ---------- Robot Subsystems ---------- \\
  private final Drivetrain drive = new Drivetrain();
  // private final Elevator elevator = new Elevator();

  // The robot's controllers
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;

  private final Arm arm = new Arm();
  // private final FourBarArm fourBarArm = new FourBarArm();

  private final Claw claw = new Claw();
  private final ElevatorPID elevatorPID = new ElevatorPID();

  // Create a Sendable Chooser, which allows us to select between Commands (in
  // this case, auto commands)
  private final SendableChooser<Command> chooser = new SendableChooser<Command>();


  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Instantiate our controllers with proper ports.
    this.xboxDriver = new XboxController(3);
    this.xboxOperator = new XboxController(2);

    // Shuffleboard.getTab("SmartDashboard")
    // .add("Reset Arm", new InstantCommand(() -> this.arm.reset()));

    // Controler Throttle Mappings
    this.drive.setDefaultCommand(
        new Drive(drive, xboxDriver));

    // this.elevator.setDefaultCommand(
    // new ElevatorController(elevator, xboxOperator)); // added, works

    this.arm.setDefaultCommand(
    new ArmController(arm, xboxOperator));

    // this.fourBarArm.setDefaultCommand(
    // new FourBarArmController(fourBarArm, xboxOperator));

    this.xboxOperator.getAButton().onTrue(
        new InstantCommand(() -> this.claw.toggle()));

    this.xboxOperator.getYButton().onTrue(
        new ElevatorUp(elevatorPID));
    this.xboxOperator.getXButton().onTrue(
      new ElevatorDown(elevatorPID));
    // testc.b().onTrue(new InstantCommand(() -> this.claw.toggle()));
    this.xboxOperator.getRightBumper().onTrue(
      new ElevatorCancel(elevatorPID));
    this.elevatorPID.setDefaultCommand(
        new ElevatorPIDController(elevatorPID, xboxOperator)); // added, untested

    // this.claw.setDefaultCommand(
    // new ClawController(claw, xboxOperator)
    // );

    // this.xboxOperator.getAButton().onTrue(
    // new InstantCommand(() -> this.claw.toggle())
    // );

    Shuffleboard.getTab("Commands").add("Zero Elevator Position",
        new InstantCommand(() -> this.elevatorPID.zeroEncoder()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // Create a voltage constraint to ensure we don't accelerate too fast
  var autoVoltageConstraint =
  new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
          Drivetrain.kS),
      DriveConstants.kDriveKinematics,
      10);

// Create config for trajectory
TrajectoryConfig config =
  new TrajectoryConfig(
          AutoConstants.kMaxSpeedMetersPerSecond,
          AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics)
      // Apply the voltage constraint
      .addConstraint(autoVoltageConstraint);

// An example trajectory to follow.  All units in meters.
Trajectory exampleTrajectory =
  TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      // Pass config
      config);

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    RamseteCommand ramseteCommand =
      new RamseteCommand(
          exampleTrajectory,
          drive::getPose,
          new RamseteController(kRamseteB, kRamseteZeta),
          new SimpleMotorFeedforward(
              Drivetrain.ksVolts,
              Drivetrain.kvVoltSecondsPerMeter,
              Drivetrain.kaVoltSecondsSquaredPerMeter),
              Drivetrain.m_kinematics,
              drive::getWheelSpeeds,
          new PIDController(DriveConstants.kPDriveVel, 0, 0),
          new PIDController(DriveConstants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          drive::tankDriveVolts,
          drive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

  public Command getAutonomousCommand() {
    Command chosen = chooser.getSelected();
    return chosen;
  }
  
  
}
