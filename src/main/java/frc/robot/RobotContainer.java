// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.net.URI;
import java.nio.file.Files;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.RebelUtil;
import frc.lib.input.XboxController;
import frc.robot.commands.ElevatorCancel;
import frc.robot.commands.AutoAlign;
// import frc.robot.commands.ArmPositionSet;
// import frc.robot.commands.ArmUp;
import frc.robot.commands.AutoBalance;
// import frc.robot.subsystems.ArmPID;
// import frc.robot.commands.ArmPIDController;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorPIDController;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.FalconDrive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ElevatorPIDNonProfiled;
import frc.robot.subsystems.FalconDrivetrain;
import frc.robot.subsystems.LinSlidePID;
import frc.robot.subsystems.LinSlidePiston;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.AutoRunner;
import frc.robot.commands.TurretController;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.PoseEstimator;
// import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.WristController;
import frc.robot.commands.WristDown;
import frc.robot.commands.WristStraight;
import frc.robot.commands.WristTurtle;
import frc.robot.commands.WristUp;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.commands.ElevatorDownLinSlideIn;
import frc.robot.commands.ElevatorGetFromLoading;
import frc.robot.commands.ElevatorUpLinSlideOut;
import frc.robot.commands.LinSlidePIDController;
import frc.robot.commands.LinSlideToggle;
import frc.robot.commands.MidScore;
import frc.robot.commands.Place;
import frc.robot.commands.TimerCommand;
import frc.robot.commands.ToPickup;
import frc.robot.commands.LinSlideFullyIn;
import frc.robot.commands.LinSlideFullyOut;
import frc.robot.subsystems.LinSlidePID;
import frc.robot.commands.AutoNotch;

import frc.robot.utils.ConstantsArmElevator.ElevatorConstants;
import frc.robot.utils.ConstantsArmElevator.ArmConstants;

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
  // ---------- Robot Subsystems ---------- \\

  public static RobotContainer instance = null;
  // The robot's controllers
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;
  private final XboxController xboxTester;

  private final Wrist wrist = Wrist.getInstance();
  private final ElevatorPIDNonProfiled elevatorFinal = ElevatorPIDNonProfiled.getInstance();
  // private final Turret turret = Turret.getInstance();
  private final LinearSlide linslide = LinearSlide.getInstance();
  private final LinSlidePiston LinPiston = LinSlidePiston.getInstance();
  private final Claw claw = Claw.getInstance();
  private final FalconDrivetrain drive = FalconDrivetrain.getInstance();
  private final AutoRunner auto = AutoRunner.getInstance();
  // private final PoseEstimator poseEstimator = PoseEstimator.getInstance();

  // private final Drivetrain drive = new Drivetrain();
  // private final Arm arm = Arm.getInstance();
  // private final LinSlidePID linslidePID = new LinSlidePID();
  // private final ArmPID armPID = new ArmPID();
  // private final ElevatorPID elevatorPID = ElevatorPID.getInstance();
  // DO NOT RUN ELEVATOR WITHOUT ZEROING ENCODERS AT GROUND POSITION YOU WILL
  // BREAK IT IF YOU DONT DO THIS
  // private final Elevator elevator = new Elevator();

  // Create a Sendable Chooser, which allows us to select between Commands (in
  // this case, auto commands)

  public RobotContainer() {
    // Instantiate our controllers with proper ports.
    this.xboxDriver = new XboxController(3);
    this.xboxOperator = new XboxController(2);
    this.xboxTester = new XboxController(1);

    // Controller Throttle Mappings
    this.drive.setDefaultCommand(new FalconDrive(drive, xboxDriver));

    // Run a linslide in command to start the match
    // (new LinSlideFullyIn(linslide, LinPiston)).schedule();

    this.elevatorFinal.setDefaultCommand(new ElevatorPIDController(elevatorFinal, xboxOperator));
    this.wrist.setDefaultCommand(new WristController(wrist, xboxOperator));
    this.xboxOperator.getRightBumper().onTrue(new SequentialCommandGroup(
        new Place(),
        new ElevatorDownLinSlideIn()));
    this.xboxOperator.getLeftBumper().onTrue(new LinSlideToggle(linslide, LinPiston));

    // presets
    this.xboxOperator.getYButton().onTrue(new ElevatorGetFromLoading());
    this.xboxOperator.getXButton().onTrue(new ElevatorDownLinSlideIn());
    this.xboxOperator.getBButton().onTrue(new ElevatorUpLinSlideOut());
    this.xboxOperator.getAButton().onTrue(new InstantCommand(() -> this.claw.toggle()));
    this.xboxOperator.getLeftMiddleButton().onTrue(new WristDown(Wrist.getInstance()));
    this.xboxOperator.getRightMiddleButton().onTrue(new MidScore());

    // toggle gear
    this.xboxDriver.getRightBumper().onTrue(new InstantCommand(() -> this.drive.switchToHighGear()));
    this.xboxDriver.getLeftBumper().onTrue(new InstantCommand(() -> this.drive.switchToLowGear()));
    this.xboxDriver.getYButton().whileTrue(new AutoNotch(drive));
    this.xboxDriver.getAButton().whileTrue(new AutoBalance(drive, PoseEstimator.getInstance()));
    this.xboxDriver.getBButton().onTrue(new SequentialCommandGroup(
      new Place(),
      new ElevatorDownLinSlideIn()));
    this.xboxDriver.getXButton().whileTrue(new AutoAlign(drive, PoseEstimator.getInstance()));
    this.xboxDriver.getLeftMiddleButton().onTrue(new InstantCommand(() -> wrist.zeroEncoder()));

    // this.xboxTester.getLeftBumper().onTrue(new InstantCommand(() -> this.LinPiston.push()));
    // this.xboxTester.getRightBumper().onTrue(new InstantCommand(() -> this.LinPiston.pull()));
    
    // this.turret.setDefaultCommand(new TurretController(turret, xboxOperator));

    // this.xboxOperator.getUpDpad().onTrue(new InstantCommand(() ->
    // this.turret.setGoal(0)));
    // this.xboxOperator.getLeftDpad().onTrue(new InstantCommand(() ->
    // this.turret.setGoal(-0.0526))); // around 3 deg, 3/57 rad
    // this.xboxOperator.getRightDpad().onTrue(new InstantCommand(() ->
    // this.turret.setGoal(0.0526))); // around 3 deg, 3/57 rad

    // MAKE SURE TO REBOOT RIO TO FIX MEMORY ISSUES
    // System.gc();
    // Runtime.getRuntime().freeMemory();

    // this.arm.setDefaultCommand(new ArmController(arm, xboxOperator));
    // this.armPID.setDefaultCommand(new ArmPIDController(armPID, xboxOperator));
    // this.elevatorPID.setDefaultCommand(new ElevatorPIDController(elevatorPID,
    // xboxOperator));

    // this.linslide.setDefaultCommand(new LinearSlideController(linslide,
    // xboxOperator)); // rightX
    // this.linslidePID.setDefaultCommand(new LinSlidePIDController(linslidePID,
    // xboxOperator));

    // this.xboxOperator.getRightBumper().onTrue(new InstantCommand(() ->
    // this.LinPiston.pull()));
    // this.xboxOperator.getLeftBumper().onTrue(new InstantCommand(() ->
    // this.LinPiston.push()));

    // this.xboxDriver.getBButton().whileTrue(new InstantCommand( () -> new
    // AutoBalance(this.drive, )));

    // this.xboxOperator.getYButton().onTrue(new ElevatorUp(elevatorFinal));
    // this.xboxOperator.getXButton().onTrue(new ElevatorDown(elevatorFinal));

    Shuffleboard.getTab("Encoders").add("Zero Encoder", new InstantCommand(() -> wrist.zeroEncoder()));
    Shuffleboard.getTab("Drive").add("Zero Heading", new InstantCommand(PoseEstimator.getInstance()::resetHeading));
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
    auto.loadPath();
    return auto.getCommand();
  }

  public void resetForAuto() {
    // FalconDrivetrain.getInstance().zeroEncoder();
    PoseEstimator.getInstance().resetPitchOffset();
    ElevatorPIDNonProfiled.getInstance().zeroEncoder();
    LinearSlide.getInstance().zeroEncoder();
    Wrist.getInstance().turtleEncoder();
  }

  public void checkControllers() {
    // TODO: Uncomment this when we ensure deadband is high enough to not interfere with other commands

    double desiredVeloWrist = RebelUtil.linearDeadband(xboxOperator.getRightY(), 0.2) * Wrist.kMaxSpeed;
    if(desiredVeloWrist != 0) wrist.setToVelocityControlMode(true);
    
    double desiredVeloElev = RebelUtil.linearDeadband(xboxOperator.getLeftY(), 0.2) * ElevatorPID.kMaxSpeed;
    if(desiredVeloElev != 0) elevatorFinal.setToVelocityControlMode(true);
  }
}