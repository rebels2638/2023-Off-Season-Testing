// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.lib.input.XboxController;
import frc.robot.commands.ArmController;
// import frc.robot.commands.ArmDown;
import frc.robot.commands.Drive;
import frc.robot.commands.ElevatorCancel;
import frc.robot.commands.ElevatorController;
// import frc.robot.commands.ArmPositionSet;
// import frc.robot.commands.ArmUp;
import frc.robot.commands.Auto;
import frc.robot.commands.AutoBalance;
// import frc.robot.subsystems.ArmPID;
// import frc.robot.commands.ArmPIDController;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorPIDController;
import frc.robot.commands.ElevatorPositionSet;
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
// import frc.robot.commands.PositionPresets;
import frc.robot.commands.TurretController;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.WristController;
import frc.robot.commands.WristUp;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.commands.ElevatorDownLinSlideIn;
import frc.robot.commands.ElevatorUpLinSlideOut;
import frc.robot.commands.LinSlidePIDController;
import frc.robot.commands.LinearSlideController;
import frc.robot.commands.LinSlideFullyIn;
import frc.robot.commands.LinSlideFullyOut;
import frc.robot.subsystems.LinSlidePID;

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

  // The robot's controllers
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;

  private final PoseEstimator poseEstimator = PoseEstimator.getInstance();
  private final FalconDrivetrain drive = FalconDrivetrain.getInstance();
  private final ElevatorPIDNonProfiled elevatorFinal = ElevatorPIDNonProfiled.getInstance();
  private final Turret turret = Turret.getInstance();
  private final LinearSlide linslide = LinearSlide.getInstance();
  private final LinSlidePiston LinPiston = LinSlidePiston.getInstance();
  private final Wrist wrist = new Wrist();
  private final Claw claw = Claw.getInstance();

  
  // private final Drivetrain drive = new Drivetrain();
  // private final Arm arm = Arm.getInstance();
  // private final LinSlidePID linslidePID = new LinSlidePID();
  // private final ArmPID armPID = new ArmPID();
  // private final ElevatorPID elevatorPID = ElevatorPID.getInstance(); //DO NOT
  // RUN ELEVATOR WITHOUT ZEROING ENCODERS AT GROUND POSITION YOU WILL BREAK IT IF
  // YOU DONT DO THIS
  // private final Elevator elevator = new Elevator();

  // Create a Sendable Chooser, which allows us to select between Commands (in
  // this case, auto commands)
  private final SendableChooser<Command> chooser = new SendableChooser<Command>();
  private final SendableChooser<String> pathChooser = new SendableChooser<String>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Instantiate our controllers with proper ports.
    this.xboxDriver = new XboxController(3);
    this.xboxOperator = new XboxController(2);

    // MAKE SURE TO REBOOT RIO TO FIX MEMORY ISSUES
    // System.gc();
    // Runtime.getRuntime().freeMemory();

    // Controller Throttle Mappings
    this.drive.setDefaultCommand(
        new FalconDrive(drive, xboxDriver));

    // this.drive.setDefaultCommand
    // new Drive(drive, xboxDriver));

    this.wrist.setDefaultCommand(new WristController(wrist, xboxOperator));
    // this.arm.setDefaultCommand(
    // new ArmController(arm, xboxOperator));
    // this.elevator.setDefaultCommand(
    // new ElevatorController(elevator, xboxOperator)); // added, works

    // this.linslide.setDefaultCommand(
    // new LinearSlideController(linslide, xboxOperator)); // rightX

    // this.linslidePID.setDefaultCommand(
    // new LinSlidePIDController(linslidePID, xboxOperator)
    // );

    this.turret.setDefaultCommand(
        new TurretController(turret, xboxOperator));

    // this.elevatorPID.setDefaultCommand(
    // new ElevatorPIDController(elevatorPID, xboxOperator));

    // this.armPID.setDefaultCommand(
    // new ArmPIDController(armPID, xboxOperator));

    this.xboxOperator.getAButton().onTrue(
        new InstantCommand(() -> this.claw.toggle()));

    // this.linslide.setDefaultCommand(new LinearSlideController(linslide,
    // xboxOperator));

    // this.xboxOperator.getRightBumper().onTrue(new InstantCommand(() ->
    // this.LinPiston.pull()));
    // this.xboxOperator.getLeftBumper().onTrue(new InstantCommand(() ->
    // this.LinPiston.push()));

    // Run a linslide in command to start the match
    (new LinSlideFullyIn(linslide, LinPiston)).schedule();

    this.xboxOperator.getRightBumper().onTrue(new LinSlideFullyOut(linslide, LinPiston));
    this.xboxOperator.getLeftBumper().onTrue(new LinSlideFullyIn(linslide, LinPiston));

    // this.xboxOperator.getUpDpad().onTrue(new InstantCommand(() ->
    // this.turret.setGoal(0)));
    // this.xboxOperator.getLeftDpad().onTrue(new InstantCommand(() ->
    // this.turret.setGoal(-0.0526))); // around 3 deg, 3/57 rad
    // this.xboxOperator.getRightDpad().onTrue(new InstantCommand(() ->
    // this.turret.setGoal(0.0526))); // around 3 deg, 3/57 rad

    // this.xboxDriver.getBButton().whileTrue(
    // new InstantCommand( () -> new AutoBalance(this.drive, ))
    // );

    // this.xboxOperator.getYButton().onTrue(
    // new FourBarUp(fourBarArm)
    // );
    // this.xboxOperator.getYButton().onTrue(
    // new ElevatorUp(elevatorPID));
    // this.xboxOperator.getXButton().onTrue(
    // new ElevatorDown(elevatorPID));
    // // testc.b().onTrue(new InstantCommand(() -> this.claw.toggle()));
    // this.xboxOperator.getAButton().onTrue(
    // new ElevatorCancel(elevatorPID));
    this.elevatorFinal.setDefaultCommand(
        new ElevatorPIDController(elevatorFinal, xboxOperator)); // added, untested
    this.xboxOperator.getBButton().onTrue(new ElevatorUpLinSlideOut());
    this.xboxOperator.getXButton().onTrue(new ElevatorDownLinSlideIn());
    this.xboxOperator.getYButton().onTrue(new ElevatorUp(elevatorFinal));

    // this.xboxOperator.getYButton().onTrue(new ElevatorUp(elevatorFinal));
    // this.xboxOperator.getXButton().onTrue(new ElevatorDown(elevatorFinal));
    // this.xboxOperator.getAButton().onTrue(new PositionPresets(elevatorPID, arm,
    // linslide, turret, "loadingStation"));
    // this.xboxOperator.getBButton().onTrue(new PositionPresets(elevatorPID, arm,
    // linslide, turret, "idle"));
    // this.xboxOperator.getYButton().onTrue(new PositionPresets(elevatorPID, arm,
    // linslide, turret, "highScore"));
    // this.xboxOperator.getXButton().onTrue(new PositionPresets(elevatorPID, arm,
    // linslide, turret, "midScore"));

    // this.xboxOperator.getYButton().onTrue(
    // new ArmUp(armPID)
    // );
    // this.xboxOperator.getXButton().onTrue(
    // new ArmDown(armPID)
    // );

    // this.xboxOperator.getBButton().onTrue(
    // new ElevatorUp(elevatorPID)
    // );

    this.xboxDriver.getRightBumper().onTrue(
        new InstantCommand(() -> this.drive.switchToHighGear()));
    this.xboxDriver.getLeftBumper().onTrue(
        new InstantCommand(() -> this.drive.switchToLowGear()));

    chooser.addOption("RamseteFollower", new Auto(drive, this));
    pathChooser.addOption("hPath", "hPath.wpilib.json");

    Shuffleboard.getTab("Encoders").add("Zero Encoder", new InstantCommand(() -> wrist.zeroEncoder()));
    Shuffleboard.getTab("Auto").add("Command", chooser);
    Shuffleboard.getTab("Auto").add("Path", pathChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    Command chosen = chooser.getSelected();
    return chosen;
  }

  public String getPathFileName() {
    String chosen = pathChooser.getSelected();
    return chosen;
  }
}
