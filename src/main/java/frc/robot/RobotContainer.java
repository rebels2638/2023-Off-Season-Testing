// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.RebelUtil;
import frc.lib.input.XboxController;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ElevatorPIDNonProfiled;
import frc.robot.subsystems.FalconDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LinSlidePiston;
import frc.robot.subsystems.AutoRunner;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.Navx;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.AutoConstants.LimelightConstants;
import frc.robot.commands.auto.AutoAlign;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.AutoNotch;
import frc.robot.commands.drivetrain.FalconDrive;
import frc.robot.commands.elevator.ElevatorPIDController;
import frc.robot.commands.linslide.LinSlideToggle;
import frc.robot.commands.presets.HighScore;
import frc.robot.commands.presets.LoadingStationPickup;
import frc.robot.commands.presets.MidScore;
import frc.robot.commands.presets.Place;
import frc.robot.commands.presets.TurtleMode;
import frc.robot.commands.wrist.WristController;
import frc.robot.commands.wrist.WristDown;

import frc.robot.commands.DriveSecondary;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static RobotContainer instance = null;

  // The robot's controllers
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;
  private final XboxController xboxTester;

  public final double LEFT_X_DEADBAND = 0.01;
  public final double LEFT_Y_DEADBAND = 0.01;
  
  // Robot Subsystems
  private final Wrist wrist = Wrist.getInstance();
  // private final ElevatorPID elevator = ElevatorPID.getInstance();
  private final ElevatorPIDNonProfiled elevator = ElevatorPIDNonProfiled.getInstance();
  private final LinearSlide linSlide = LinearSlide.getInstance();
  private final LinSlidePiston linPiston = LinSlidePiston.getInstance();
  private final Claw claw = Claw.getInstance();
  private final DriveSecondary drivebase = new DriveSecondary();
  private final AutoRunner auto = AutoRunner.getInstance();
  private final Navx gyro = Navx.getInstance();
  private final PoseEstimator poseEstimator = PoseEstimator.getInstance();
  private final Limelight limelight = Limelight.getInstance();

  public RobotContainer() {
    // Instantiate our controllers with proper ports.
    this.xboxTester = new XboxController(1);
    this.xboxOperator = new XboxController(2);
    this.xboxDriver = new XboxController(3);

    // Controller Throttle Mappings
    // this.drive.setDefaultCommand(new FalconDrive(drive, limelight, xboxDriver));
    this.drive.setDefaultCommand(new DriveSecondary(drivebase,
                                                    () -> MathUtil.applyDeadband(xboxDriver.getLeftY(), LEFT_Y_DEADBAND),
                                                    () -> MathUtil.applyDeadband(xboxDriver.getRightX(), LEFT_X_DEADBAND),
                                                    () -> -xboxDriver.getRawAxis(3), () -> true, false, true)); // replace the raw axis call with something which calculates the angular velocity using the formula dθ/dt dθ can be obtained by doing some method calls from pose2D im quite sure, the rest should be trivial, the class maintains an internal timer for dt so make it ret that
    this.elevator.setDefaultCommand(new ElevatorPIDController(elevator, xboxOperator));
    this.wrist.setDefaultCommand(new WristController(wrist, xboxOperator));

    // Driver presets
    // this.xboxDriver.getRightBumper().onTrue(new InstantCommand(() -> this.drive.switchToHighGear()));
    // this.xboxDriver.getLeftBumper().onTrue(new InstantCommand(() -> this.drive.switchToLowGear()));
    this.xboxDriver.getYButton().whileTrue(new AutoNotch(drive));
    this.xboxDriver.getAButton().whileTrue(new AutoBalance(drive, poseEstimator));
    this.xboxDriver.getBButton().onTrue(new SequentialCommandGroup(
      new Place(),
      new TurtleMode()));
    this.xboxDriver.getXButton().whileTrue(new AutoAlign(drive, limelight, poseEstimator));
    this.xboxDriver.getLeftMiddleButton().onTrue(new InstantCommand(() -> wrist.zeroEncoder()));

    // Operator presets
    this.xboxOperator.getYButton().onTrue(new LoadingStationPickup());
    this.xboxOperator.getXButton().onTrue(new TurtleMode());
    this.xboxOperator.getBButton().onTrue(new HighScore());
    this.xboxOperator.getAButton().onTrue(new InstantCommand(() -> this.claw.toggle()));
    this.xboxOperator.getLeftMiddleButton().onTrue(new WristDown(Wrist.getInstance()));
    this.xboxOperator.getRightMiddleButton().onTrue(new MidScore());
    this.xboxOperator.getRightBumper().onTrue(new SequentialCommandGroup(
        new Place(),
        new TurtleMode()));
    this.xboxOperator.getLeftBumper().onTrue(new LinSlideToggle(linSlide, linPiston));
  }

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auto.getCommand();
  }

  // Reset encoders for auto
  public void resetForAuto(Pose2d pose) {
    drive.resetOdometry(pose);
    limelight.setMode(LimelightConstants.APRILTAG_PIPELINE);
    gyro.resetGyroToPose(pose);
    poseEstimator.resetPose(pose);
    elevator.zeroEncoder();
    linSlide.zeroEncoder();
    wrist.turtleEncoder();
  }

  public void prepareForAuto() {
    auto.prepareForAuto();
  }

  // Override commands and switch to manual control
  public void checkControllers() {
    double desiredVeloWrist = RebelUtil.linearDeadband(xboxOperator.getRightY(), 0.2) * Wrist.kMaxSpeed;
    if(desiredVeloWrist != 0) wrist.setToVelocityControlMode(true);
    
    double desiredVeloElev = RebelUtil.linearDeadband(xboxOperator.getLeftY(), 0.2) * ElevatorPID.kMaxSpeed;
    if(desiredVeloElev != 0) elevator.setToVelocityControlMode(true);
  }
}