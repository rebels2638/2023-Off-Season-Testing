// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GyroSubystem;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
/** An example command that uses an example subsystem. */
public class AutoBalance extends PIDCommand {
  private final Drivetrain drive_s;
  private final GyroSubystem gyro_s;

  private double error;
  private double angle;
  private double drivePower;
  private final double threshold = 0.5; // Actual guess
  private final double goal = 0;
  private final double limitPower = 0.5; 
  private final double kPConstant = 0.02; // please work, oh please work. I beg of you (Proportionality Constant of PID controller)

  public AutoBalance(Drivetrain drive, GyroSubystem gyro) {
    super(new PIDController(0.02, 0, 0), gyro::getAngle, 0, output -> drive.drive(output, output));
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(threshold, limitPower);
    this.drive_s = drive;
    this.gyro_s = gyro;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive,gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        angle = gyro_s.getAngle();
        error = goal - angle;
        drivePower = -Math.min(kPConstant * error, 1);
        drivePower = Math.min(drivePower, limitPower);
        drive_s.drive(drivePower, drivePower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        drive_s.drive(0,0);
        gyro_s.zeroGyro();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < threshold || getController().atSetpoint();
  }
}
