// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

// import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ElevatorPIDNonProfiled;
import frc.lib.RebelUtil;
import frc.lib.input.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;

/** An example command that uses an example subsystem. */
public class ElevatorPIDController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorPIDNonProfiled m_elevatorPID;
  // private final ElevatorPID m_elevatorPID;
  private final XboxController e_controller; // e_controller is elevator's controller
  
  DigitalInput toplimitSwitch = new DigitalInput(2);
  DigitalInput bottomlimitSwitch = new DigitalInput(1);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorPIDController(ElevatorPIDNonProfiled elevatorPIDSubsystem,/*ElevatorPID elevatorPIDSubsystem,*/ XboxController controller) {
    e_controller = controller;
    m_elevatorPID = elevatorPIDSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorPID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_elevatorPID.setToVelocityControlMode(true);
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double error = 0.0;
    // if (e_controller.getLeftY() < 0.0) {error = -0.1;}
    // else {error = 0.1;}

    double desiredVelo = RebelUtil.linearDeadband(e_controller.getLeftY(), 0.05) * ElevatorPID.kMaxSpeed;
    
    if(desiredVelo != 0) m_elevatorPID.setToVelocityControlMode(true);
    m_elevatorPID.setVelocitySetpoint(desiredVelo);
    //System.out.println("Controller: "+e_controller.getLeftY());
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
