package frc.robot.commands;

import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Turret;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurretFollow extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Turret m_turret;
  private final PoseEstimator m_poseEstimator;

  public TurretFollow(Turret subsystem, PoseEstimator estimator) {
    m_turret = subsystem;
    m_poseEstimator = estimator;
    addRequirements(subsystem, estimator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Find the robot pose
    Pose2d robotPose = m_poseEstimator.getCurrentPose();

    // Find the field pose of our tracked object
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
