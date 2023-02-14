// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;

import frc.robot.subsystems.FourBarArmPID;
import frc.robot.subsystems.FourBarArm;
import frc.lib.RebelUtil;
import frc.lib.input.XboxController;
import edu.wpi.first.math.controller.PIDController;

//import org.apache.commons.io.filefilter.TrueFileFilter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FourBarArmPIDController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // private final FourBarArm m_armSubsystem;
  private final FourBarArmPID m_armSubsystem;
  private final XboxController controller; // controller is arm's controller
  // private final DigitalInput toplimitSwitch;
  // private final DigitalInput bottomlimitSwitch;

  // Creates a new ExampleCommand.
  public FourBarArmPIDController(FourBarArmPID armSubsystem, XboxController controller) {
    this.controller = controller;
    m_armSubsystem = armSubsystem;
    // toplimitSwitch = new DigitalInput(2);
    // bottomlimitSwitch = new DigitalInput(1);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.setToVelocityControlMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double talonInputpercent = controller.getLeftY(); // not sure which joystick yet
    double inputPercent775 = controller.getRightY()*0.5; // not sure which joystick
    
    double desiredVelo = RebelUtil.linearDeadband(talonInputpercent, 0.05) * FourBarArmPID.kMaxSpeed;
    m_armSubsystem.setVelocitySetpoint(desiredVelo);

    // m_armSubsystem.setPercentOutputTalon(talonInputpercent); 
    // m_armSubsystem.setPercentOutput775(inputPercent775); // takes percent. trust
    
    /*
    if (inputPercent > 0) {
        if (toplimitSwitch.get()) {
            // We are going up and top limit is tripped so stop
             m_elevatorSubsystem.setPercentOutput(0);
        } else {
            // We are going up but top limit is not tripped so go at commanded speed
             m_elevatorSubsystem.setPercentOutput(inputPercent);
        }
    } else {
        if (!bottomlimitSwitch.get()) {
            // We are going down and bottom limit is tripped so stop
             m_elevatorSubsystem.setPercentOutput(0);
        } else {
            // We are going down but bottom limit is not tripped so go at commanded speed
            m_elevatorSubsystem_m.setPercentOutput(inputPercent);
        }
    }
    */
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
