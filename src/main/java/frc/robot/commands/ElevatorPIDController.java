// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorPID;
import frc.lib.input.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DigitalInput;

/** An example command that uses an example subsystem. */
public class ElevatorPIDController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorPID m_elevatorPID;
  private final XboxController e_controller; // e_controller is elevator's controller
  
  DigitalInput toplimitSwitch = new DigitalInput(2);
  DigitalInput bottomlimitSwitch = new DigitalInput(1);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorPIDController(ElevatorPID elevatorPIDSubsystem, XboxController controller) {
    e_controller = controller;
    m_elevatorPID = elevatorPIDSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorPID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
     double desiredHeight = e_controller.getLeftY();
    
    if (desiredHeight > 0) {
        if (toplimitSwitch.get()) {
            // We are going up and top limit is tripped so stop
             m_elevatorPID.setSetpoint(0);
        } else {
            // We are going up but top limit is not tripped so go at commanded speed
             m_elevatorPID.setSetpoint(desiredHeight);
        }
    } else {
        if (bottomlimitSwitch.get()) {
            // We are going down and bottom limit is tripped so stop
             m_elevatorPID.setSetpoint(0);
        } else {
            // We are going down but bottom limit is not tripped so go at commanded speed
             m_elevatorPID.setSetpoint(desiredHeight);
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorPID.m_controller.atGoal();
  }
}
