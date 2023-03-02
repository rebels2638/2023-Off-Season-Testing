// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Turret;
import frc.lib.RebelUtil;
import frc.lib.input.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class TurretFollow extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Turret m_turretSubsystem;
  private final PoseEstimator poseEstimator;
  private final Pose2d target;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurretFollow(Turret turretSubsystem, PoseEstimator poseEstimatorSubsystem, Pose2d target) {
    m_turretSubsystem = turretSubsystem;
    poseEstimator = poseEstimatorSubsystem;
    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSubsystem);
    addRequirements(poseEstimatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_turretSubsystem.m_velocityControlEnabled = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Pose2d currentPos = poseEstimator.getCurrentPose();
    double goal = Math.atan((Math.abs(target.getY() - currentPos.getY())) / (Math.abs(target.getX() - currentPos.getX())));
    m_turretSubsystem.setGoal(goal);
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
