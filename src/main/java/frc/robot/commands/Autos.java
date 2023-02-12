// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FalconDrivetrain;
import frc.robot.utils.ConstantsFXDriveTrain.DriveConstants;

import java.util.Arrays;
import java.util.function.Consumer;

import com.pathplanner.lib.*;

public final class Autos
    extends CommandBase {
  private FalconDrivetrain m_drivetrain;
  private boolean finished = false;
  private ShuffleboardTab tab = Shuffleboard.getTab("Auto");
  private PathPlannerTrajectory pl;

  public Autos(FalconDrivetrain drive) {
    m_drivetrain = drive;
    addRequirements(drive);
    SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(
        DriveConstants.ksVolts,
        DriveConstants.kvVoltSecondsPerMeter,
        DriveConstants.kaVoltSecondsSquaredPerMeter);
    tab.add("Auto Selection: ", drive);

  }

  public static final PathPlannerTrajectory[] AUTO_PATHS = {
      PathPlanner.generatePath(new PathConstraints(4, 5), Arrays.asList(
          new PathPoint[] {
              new PathPoint(new Translation2d(10D, 0D), Rotation2d.fromDegrees(300)),
              new PathPoint(new Translation2d(5D, 10D), Rotation2d.fromDegrees(15)),
          }))
  };

  public void load(final String fileName, PathConstraints defaultConstraints) {
    this.pl = PathPlanner.loadPath(fileName, defaultConstraints);
  }

  public static PathPoint make(double x, double y, double angle_theta) {
    return new PathPoint(new Translation2d(x, y), Rotation2d.fromDegrees(angle_theta));
  }

  public static PathPlannerTrajectory make(PathConstraints constraints, PathPoint... points) {
    return PathPlanner.generatePath(constraints, Arrays.asList(points));
  }

  @Override
  public void execute() {
    this.pl.getStates().forEach(x -> {

    });
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
