// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.lib.input.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ElevatorController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator m_elevatorSubsystem;
  private final XboxController e_controller; // e_controller is elevator's controller
  private final DigitalInput toplimitSwitch;
  private final DigitalInput bottomlimitSwitch;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorController(Elevator elevatorSubsystem, XboxController controller) {
    e_controller = controller;
    m_elevatorSubsystem = elevatorSubsystem;
    toplimitSwitch = new DigitalInput(0);
    bottomlimitSwitch = new DigitalInput(1);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (e_controller.getLeftY() > 0) {
        if (toplimitSwitch.get()) {
            // We are going up and top limit is tripped so stop
             m_elevatorSubsystem(0);
        } else {
            // We are going up but top limit is not tripped so go at commanded speed
             m_elevatorSubsystem(e_controller.getLeftY()*0.5);
        }
    } else {
        if (bottomlimitSwitch.get()) {
            // We are going down and bottom limit is tripped so stop
             m_elevatorSubsystem(0);
        } else {
            // We are going down but bottom limit is not tripped so go at commanded speed
             m_elevatorSubsystem(e_controller.getLeftY()*0.5);
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
