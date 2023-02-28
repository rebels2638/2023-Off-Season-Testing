// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.lib.input.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ElevatorController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator m_elevatorSubsystem;
  private final XboxController e_controller; // e_controller is elevator's controller
  private boolean done;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorController(Elevator elevatorSubsystem, XboxController controller) {
    e_controller = controller;
    m_elevatorSubsystem = elevatorSubsystem;
    done = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("ELEVATOR VOLTAGE", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // fix this once we invert the motor pls

    /*
    if (!done) {

      m_elevatorSubsystem.setPercentOutput(0.5); 
      m_elevatorSubsystem.resetEncoder();

      if (m_elevatorSubsystem.getLimitSwitch()) {done = true;}
    
    }*/

    double inputPercent = e_controller.getLeftY() * 0.5;

    // if (inputPercent > 0) {
    //     if (toplimitSwitch.get()) {
    //         // We are going up and top limit is tripped so stop
    //          m_elevatorSubsystem.setPercentOutput(0);
    //     } else {
    //         // We are going up but top limit is not tripped so go at commanded speed
    //          m_elevatorSubsystem.setPercentOutput(inputPercent);
    //     }
    // } else {
    //     if (!bottomlimitSwitch.get()) {
    //         // We are going down and bottom limit is tripped so stop
    //          m_elevatorSubsystem.setPercentOutput(0);
    //     } else {
    //         // We are going down but bottom limit is not tripped so go at commanded speed
    //         m_elevatorSubsystem.setPercentOutput(inputPercent);
    //     }
    // }

    
    m_elevatorSubsystem.setVoltage(SmartDashboard.getNumber("ELEVATOR VOLTAGE", 0) + inputPercent);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
