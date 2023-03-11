package frc.robot.commands;

import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ExampleSubsystem;
//import frc.robot.subsystems.LinSlidePID;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class PositionPresets extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorPID m_elevatorSubsystem;
  private final Wrist m_wristSubsystem;
  private final LinearSlide m_linslideSubsystem;
  private final Turret m_turretSubsystem;

  //hi kush :)
  private double kHeightPositionElevator; // meters
  private double kWristAngle; // radians
  private double kExtensionOutLinSlide; // dunno
  private double kTurretAngle; // radians
  private TrapezoidProfile.State kGoalStateElevator;
  private TrapezoidProfile.State kGoalStateLinSlide;

  public PositionPresets(ElevatorPID sub1, Wrist sub2, LinearSlide sub3, Turret sub4, String preset) {

    // new TrapezoidProfile.State(kHeightPositionElevator, 0.0);

    if (preset.equals("midScore")) {

        kHeightPositionElevator = 0.0; // meters
        kWristAngle = 0.0; // radians
        kExtensionOutLinSlide = 0.0; // dunno
        kTurretAngle = 0.0; // radians
        kGoalStateElevator = new TrapezoidProfile.State(kHeightPositionElevator, 0.0);
        kGoalStateLinSlide = new TrapezoidProfile.State(kHeightPositionElevator, 0.0);

    }

    else if (preset.equals("highScore")) {

        kHeightPositionElevator = 6.0; // meters
        kWristAngle = 0.0; // radians
        kExtensionOutLinSlide = 0.0; // dunno
        kTurretAngle = 0.0; // radians
        kGoalStateElevator = new TrapezoidProfile.State(kHeightPositionElevator, 0.0);
        kGoalStateLinSlide = new TrapezoidProfile.State(kHeightPositionElevator, 0.0);

    }

    else if (preset.equals("loadingStation")) {  // untested, completed

        kHeightPositionElevator = 0.9652; // meters
        kWristAngle = 0.0; // radians
        kExtensionOutLinSlide = 0.0; // dunno
        kTurretAngle = 0.0; // radians
        kGoalStateElevator = new TrapezoidProfile.State(kHeightPositionElevator, 0.0);
        kGoalStateLinSlide = new TrapezoidProfile.State(kHeightPositionElevator, 0.0);

    }

    else if (preset.equals("idle")) { // untested, completed

        kHeightPositionElevator = 0.0; // meters
        kWristAngle = 1.5; // radians
        kExtensionOutLinSlide = 0.0; // dunno
        kTurretAngle = 0.0; // radians
        kGoalStateElevator = new TrapezoidProfile.State(kHeightPositionElevator, 0.0);
        kGoalStateLinSlide = new TrapezoidProfile.State(kHeightPositionElevator, 0.0);

    }

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

    // m_linslideSubsystem.setToVelocityControlMode(false);
    // m_linslideSubsystem.setGoal(kGoalStateLinSlide);

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
    //m_linslideSubsystem.setToVelocityControlMode(true);
    m_turretSubsystem.setToVelocityControlMode(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turretSubsystem.atGoal() || m_turretSubsystem.m_velocityControlEnabled;
  }
}
