// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FalconDrivetrain;
import frc.robot.subsystems.PoseEstimator;

import com.ctre.phoenix.CANifier.PWMChannel;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** An example command that uses an example subsystem. */
public class AutoBalance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FalconDrivetrain m_driveTrain;
  //private final SerialPort sPort = new SerialPort(200, Port.kUSB);

  private final AHRS gyro = new AHRS(Port.kUSB);
  private final PoseEstimator poseEstimatorSubsystem;
  private final double yawErrorMargin = 5;
  private final double pitchErrorMargin = 3;
  
  private final double rkp = 1;
  private final double rki = 0;
  private final double rkd = 1;

  private double dkp = SmartDashboard.getNumber("", 1);
  private double dki = 0;
  private double dkd = 1;
  private double m_headingSetpoint;
  private boolean bBalanced = false;

  private ProfiledPIDController rpidController;
  private ProfiledPIDController dpidController;

  private ShuffleboardTab tab;

  private final GenericEntry CurrentPitch;
  private final GenericEntry headingSetPoint;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoBalance(FalconDrivetrain drive, PoseEstimator pose) {
    m_driveTrain = drive;
    poseEstimatorSubsystem = pose;
    rpidController = new ProfiledPIDController(rkp, rki, rkd, new TrapezoidProfile.Constraints(5, 1));
    dpidController = new ProfiledPIDController(dkp, dki, dkd, new TrapezoidProfile.Constraints(.1, 1));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    addRequirements(pose);


    tab = Shuffleboard.getTab("AutoBalance");
    CurrentPitch = tab.add("Current_pitch",0.0).getEntry();
    headingSetPoint = tab.add("heading_SetPoint", 0.0).getEntry();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Balance kp", 0);
    SmartDashboard.putNumber("Balance kd", 0);
    rpidController.setTolerance(yawErrorMargin);
    dpidController.setTolerance(pitchErrorMargin);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dkp = SmartDashboard.getNumber("Balance kp", 0);
    dkd = SmartDashboard.getNumber("Balance kd", 0);
    // Pose2d currentPose = poseEstimatorSubsystem.getCurrentPose();
    // double currentRot = currentPose.getRotation().getRadians();
    // m_headingSetpoint = 0.0;
    
    // if(Math.cos(currentRot) < 0.0) m_headingSetpoint = Math.PI;
    // rpidController.setGoal(m_headingSetpoint);
    // dpidController.setGoal(0);
    // if ( rpidController.atGoal() ){
    //     m_driveTrain.drive(0, rpidController.calculate(currentRot));
    // }
    // else if (dpidController.atGoal()){
    //     float currentPitch = gyro.getPitch();
    //     m_driveTrain.drive( (m_headingSetpoint == 0 ? 1 : -1) * dpidController.calculate( (double) currentPitch), 0 );
    //   }
    // else {
    //   bBalanced = true;
    // }
    // updateShuffleboard();
    // System.out.println(gyro.getRoll());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bBalanced;
  }
  void updateShuffleboard(){
    CurrentPitch.setDouble(gyro.getPitch());
    headingSetPoint.setDouble(m_headingSetpoint);
  }
}

