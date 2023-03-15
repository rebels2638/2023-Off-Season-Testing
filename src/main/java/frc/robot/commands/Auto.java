// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.FalconDrivetrain;
import frc.robot.subsystems.LinSlidePiston;
import frc.robot.subsystems.LinearSlide;
import frc.robot.utils.AutoConstants;
import frc.robot.utils.ConstantsFXDriveTrain.DriveConstants;

import java.util.Map;
import java.util.HashMap;
import java.util.Arrays;


import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

public final class Auto
    extends CommandBase {

  private static final Map<String, Command> PATH_COMMANDS = new HashMap<>();
  static {
    PATH_COMMANDS.put("linPistonClose", new LinSlideFullyIn(LinearSlide.getInstance(), LinSlidePiston.getInstance()));
    PATH_COMMANDS.put("linPistonOpen", new LinSlideFullyOut(LinearSlide.getInstance(), LinSlidePiston.getInstance()));
    PATH_COMMANDS.put("clawOpen", new InstantCommand(Claw.getInstance()::push));
    PATH_COMMANDS.put("clawClose", new InstantCommand(Claw.getInstance()::pull));
    PATH_COMMANDS.put("resetDTEncoders", new InstantCommand(FalconDrivetrain.getInstance()::resetEncoders));

  }

  private FalconDrivetrain m_drive;
  private RobotContainer m_robot;
  private boolean finished = false;
  private ShuffleboardTab tab = Shuffleboard.getTab("Auto");
  private PathPlannerTrajectory m_path;
  private RamseteAutoBuilder m_autoBuilder;
  private Command m_autoCommand;

  public Auto(FalconDrivetrain drive, RobotContainer robot) {
    m_drive = drive;
    m_robot = robot;
    m_autoBuilder = new RamseteAutoBuilder(
      drive::getPose,
      drive::resetOdometry,
      new RamseteController(),
      drive.m_kinematics,
      drive.m_feedforward,
      drive::getWheelSpeeds,
      new PIDConstants(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
      drive::setVoltageFromAuto,
      PATH_COMMANDS,
      true,
      m_drive
    );
    
    PathPlannerServer.startServer(5811);

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    // m_path = PathPlanner.loadPath(m_robot.getPathFileName(), new PathConstraints(9, 3));
    m_path = PathPlanner.loadPath("hPath", new PathConstraints(2, 0.25));
    m_autoCommand = m_autoBuilder.fullAuto(m_path);
    m_autoCommand.schedule();
  }

  @Override
  public void end(boolean interrupted) {
      m_autoCommand.cancel();
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
