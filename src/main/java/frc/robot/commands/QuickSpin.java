// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FalconDrivetrain;
import frc.lib.RebelUtil;
import frc.lib.input.XboxController;

import javax.swing.text.StyledEditorKit.BoldAction;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class QuickSpin extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FalconDrivetrain m_driveSubsystem;
  private PIDController pid = new PIDController(1, 0, 0);
  private boolean done = false;
  private double angle;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public QuickSpin(FalconDrivetrain driveSubsystem) {
    m_driveSubsystem = driveSubsystem;

    angle = m_driveSubsystem.getHeading();

    pid.setTolerance(5);
    


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_driveSubsystem.drive(0, pid.calculate(m_driveSubsystem.getHeading(), angle));
    if (pid.atSetpoint()) {
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
