// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LinearSlide;
import frc.lib.RebelUtil;
import frc.lib.input.XboxController;

//import org.apache.commons.io.filefilter.TrueFileFilter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LinearSlideController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LinearSlide m_linearSlide;
  //private final FourBarArmPID m_armSubsystem;
  private final XboxController controller; // controller is arm's controller
  // private final DigitalInput toplimitSwitch;
  // private final DigitalInput bottomlimitSwitch;

  // Creates a new ExampleCommand.
  public LinearSlideController(LinearSlide linslide, XboxController controller) {
    this.controller = controller;
    m_linearSlide = linslide;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_linearSlide);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double inputPercent = controller.getLeftY(); // not sure which joystick yet
    m_linearSlide.setPercentOutput(inputPercent);
    
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
