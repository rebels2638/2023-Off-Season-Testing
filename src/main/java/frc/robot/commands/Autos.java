// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.Arrays;
import java.util.function.Consumer;

import com.pathplanner.lib.*;

public final class Autos 
    extends CommandBase
{
  private boolean isFinished = false, run = false;
  private PathPlannerTrajectory pl;
  private final Drivetrain drive;
  private Consumer<Pose2d> user;

  public static final PathPlannerTrajectory[] AUTO_PATHS = 
  {
    PathPlanner.generatePath(new PathConstraints(4, 5), Arrays.asList(
      new PathPoint[]
      {
        new PathPoint(new Translation2d(10D, 0D), Rotation2d.fromDegrees(300)),
        new PathPoint(new Translation2d(5D, 10D), Rotation2d.fromDegrees(15)),
      }
    ))
  };

  public void load(final String fileName, PathConstraints defaultConstraints)
  {
    this.pl = PathPlanner.loadPath(fileName, defaultConstraints);
  }

  public static PathPoint make(double x, double y, double angle_theta)
  {
    return new PathPoint(new Translation2d(x, y), Rotation2d.fromDegrees(angle_theta));
  }

  public static PathPlannerTrajectory make(PathConstraints constraints, PathPoint... points)
  {
    return PathPlanner.generatePath(constraints, Arrays.asList(points));
  }

  // default init, supplies a default drive train to use
  public Autos(Drivetrain drivetrain, Consumer<Pose2d> worker)
  {
    this.drive = drivetrain;
    this.user = worker;
    pl = make(new PathConstraints(0,0), make(0, 0, 0));
    addRequirements(this.drive);
  }

  public Autos(Drivetrain drivetrain)
  {
    this(drivetrain, x -> {
      drivetrain.drive(0, 0);
    });
  }

  @Override public void execute()
  {
    pl.getStates().forEach(x -> {
      if(run)
      {
        user.accept(x.poseInMeters);
      }
    });
  }

  @Override public boolean isFinished()
  {
    return isFinished;
  }
}
