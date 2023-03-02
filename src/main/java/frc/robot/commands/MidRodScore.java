package frc.robot.commands;

import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LinSlidePID;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MidRodScore extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorPID m_elevatorSubsystem;
  private final Wrist m_wristSubsystem;
  private final LinSlidePID m_linslideSubsystem;
  private final Turret m_turretSubsystem;

  private final double kHeightPositionElevator = 0.381; // meters
  private final double kWristAngle = 0; // radians
  private final double kExtensionOutLinSlide = 0; // dunno
  private final double kTurretAngle = 0; // radians
  private final TrapezoidProfile.State kGoalStateElevator = new TrapezoidProfile.State(kHeightPositionElevator, 0.0);
  private final TrapezoidProfile.State kGoalStateLinSlide = new TrapezoidProfile.State(kExtensionOutLinSlide, 0.0);

  public MidRodScore(ElevatorPID sub1, Wrist sub2, LinSlidePID sub3, Turret sub4, String mode) {

    if (mode.equals("midScore")) {
        
    }

    else if (mode.equals("highScore")) {}

    else if (mode.equals("loadingStation")) {}

    else if (mode.equals("idle")) {}

    m_elevatorSubsystem = sub1;
    m_wristSubsystem = sub2;
    m_linslideSubsystem = sub3;
    m_turretSubsystem = sub4;
    
    addRequirements(sub1,sub2,sub3,sub4);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // follow position control to goal state
    m_elevatorSubsystem.setToVelocityControlMode(false);
    m_elevatorSubsystem.setGoal(kGoalStateElevator);

    m_wristSubsystem.setToVelocityControlMode(false);
    m_wristSubsystem.setGoal(kWristAngle);

    m_linslideSubsystem.setToVelocityControlMode(false);
    m_linslideSubsystem.setGoal(kGoalStateLinSlide);

    m_turretSubsystem.setToVelocityControlMode(false);
    m_turretSubsystem.setGoal(kTurretAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.setToVelocityControlMode(true);
    m_wristSubsystem.setToVelocityControlMode(true);
    m_linslideSubsystem.setToVelocityControlMode(true);
    m_turretSubsystem.setToVelocityControlMode(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turretSubsystem.atGoal() || m_turretSubsystem.m_velocityControlEnabled;
  }
}
