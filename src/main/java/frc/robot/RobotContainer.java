// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.input.XboxController;
import frc.robot.commands.ArmController;
import frc.robot.commands.ArmDown;
import frc.robot.commands.ClawController;
import frc.robot.commands.Drive;
import frc.robot.commands.ElevatorCancel;
import frc.robot.commands.ElevatorController;
import frc.robot.commands.ArmPositionSet;
import frc.robot.commands.ArmUp;
import frc.robot.subsystems.ArmPID;
import frc.robot.commands.ArmPIDController;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorPIDController;
import frc.robot.commands.ElevatorPositionSet;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.FalconDrive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.commands.FourBarUp;
import frc.robot.subsystems.FalconDrivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.commands.FourBarArmController;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.PoseEstimator;

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
//   private final Drivetrain drive = new Drivetrain();
  private final FalconDrivetrain drive = new FalconDrivetrain();
//   private final PoseEstimator poseEstimator = new PoseEstimator(drive);
  // private final Elevator elevator = new Elevator();

  // The robot's controllers
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;

//   private final Arm arm = new Arm(); 
  private final FourBarArm fourBarArm = new FourBarArm();
//   private final ArmPID armPID = new ArmPID(); 

  private final Claw claw = new Claw();
  // private final ElevatorPID elevatorPID = new ElevatorPID(); //DO NOT RUN ELEVATOR WITHOUT ZEROING ENCODERS AT GROUND POSITION YOU WILL BREAK IT IF YOU DONT DO THIS

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

    // Controller Throttle Mappings
    this.drive.setDefaultCommand(
        new FalconDrive(drive, xboxDriver));

    // this.drive.setDefaultCommand(
        // new Drive(drive, xboxDriver));


    // this.elevator.setDefaultCommand(
    // new ElevatorController(elevator, xboxOperator)); // added, works

    // this.arm.setDefaultCommand(
    // new ArmCon   troller(arm, xboxOperator));
    
    this.fourBarArm.setDefaultCommand(
    new FourBarArmController(fourBarArm, xboxOperator));

    // this.elevatorPID.setDefaultCommand(
    //     new ElevatorPIDController(elevatorPID, xboxOperator)); // added, untested

    // this.armPID.setDefaultCommand(
    // new ArmPIDController(armPID, xboxOperator));

    // this.claw.setDefaultCommand(
    // new ClawController(claw, xboxOperator)
    // );

    this.xboxOperator.getRightBumper().onTrue(
        new InstantCommand(() -> this.claw.toggle()));


    this.xboxOperator.getYButton().onTrue(
        new FourBarUp(fourBarArm)
    );    
    // this.xboxOperator.getYButton().onTrue(
    //     new ElevatorUp(elevatorPID));
    // this.xboxOperator.getXButton().onTrue(
    //   new ElevatorDown(elevatorPID));
    // // testc.b().onTrue(new InstantCommand(() -> this.claw.toggle()));
    // this.xboxOperator.getRightBumper().onTrue(
    //   new ElevatorCancel(elevatorPID));
    // this.elevatorPID.setDefaultCommand(
    //     new ElevatorPIDController(elevatorPID, xboxOperator)); // added, untested

    
    // Driving
    /*
    this.xboxOperator.getBButton().onTrue(
        new SequentialCommandGroup(
            new ArmPositionSet(armPID, ArmConstants.midScore),
            new ElevatorDown(elevatorPID)
        )
    );
    */

    
    // Ground
    // this.xboxOperator.getAButton().onTrue(
    //     new ParallelCommandGroup(
    //         new ArmPositionSet(armPID, -Math.PI / 4),
    //         new ElevatorPositionSet(elevatorPID, -0.05)
    //     )
    // );

    // // High Position
    // this.xboxOperator.getBButton().onTrue(
    //     new ParallelCommandGroup(
    //         new ArmPositionSet(armPID, 0),
    //         new ElevatorPositionSet(elevatorPID, 0.51)
    //     )
    // );

    // // Loading Station
    // this.xboxOperator.getYButton().onTrue(
    //     new ParallelCommandGroup(
    //         new ArmPositionSet(armPID, Math.PI / 4),
    //         new ElevatorPositionSet(elevatorPID, 0.51)
    //     )
    // );
    

    // this.xboxOperator.getYButton().onTrue(
    //     new ArmUp(armPID)
    // );
    // this.xboxOperator.getXButton().onTrue(
    //     new ArmDown(armPID)
    // );
    
    // this.xboxOperator.getBButton().onTrue(
    //     new ElevatorUp(elevatorPID)
    // );
    // this.xboxOperator.getAButton().onTrue(
    //     new ElevatorDown(elevatorPID)
    // );
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
}
