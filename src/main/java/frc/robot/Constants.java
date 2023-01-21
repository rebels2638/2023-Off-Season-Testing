// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DrivetrainConstants {
    public static final int kLeftLeaderId = 1;
    public static final int kLeftFollowerId = 2;
    public static final int kRightLeaderId = 3;
    public static final int kRightFollowerId = 4;

    public static final int kLeftEncoderA = 0;
    public static final int kLeftEncoderB = 1;
    public static final int kRightEncoderA = 2;
    public static final int kRightEncoderB = 3;

    public static final double kWheelRadius = 0.0508; // meters
    public static final double kTrackWidth = 0.381 * 2; // meters
    public static final int kEncoderResolution = 4096;
    public static final double kMaxSpeed = 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
    public static final double kDrivePowerLimit = 0.5; // limit power of the drivetrain
    }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class GyroConstants {
    public static final int kGyroPort = 0;
    public static final double kThreshold = 0.5;
    public static final double kGoal = 0;
    public static final double kLimitPower = 0.5; 
    public static final double kPConstant = 0.02;
  }
  public static class ClawConstants{
    public static final int kClawId = 6;
  }
  public static class ElevatorPIDConstants{
    public static final double kP = 0.00045;
    public static final double kI = 0.0015;
    public static final double kD = 0.00002;
  }

}
