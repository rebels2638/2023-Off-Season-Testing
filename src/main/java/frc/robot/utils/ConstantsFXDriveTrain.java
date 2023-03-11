// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class ConstantsFXDriveTrain {
    public static final int XBOX_OPERATOR_PORT = 2;
    public static final int XBOX_DRIVER_PORT = 3;

    public static final double FALCON_CPR = 2048;

    public static final class DriveConstants {
        public static enum DriveTypes {
            CURVATURE, ARCADE, TANK
        };

        // Left gearbox
        public static final int FALCON_LEFT_FRONT_ID = 13; // 13 1
        public static final int FALCON_LEFT_BACK_ID = 12; // 12 2
        public static final boolean FALCON_LEFT_GROUP_INVERTED = false;

        // Right gearbox
        public static final int FALCON_RIGHT_FRONT_ID = 15; // 14 3
        public static final int FALCON_RIGHT_BACK_ID = 14; // 15 4
        public static final boolean FALCON_RIGHT_GROUP_INVERTED = true;

        // Solenoid
        public static final int PCM_ID = 0;
        public static final int SOLENOID_FORWARD = 0;
        public static final int SOLENOID_REVERSE = 7;

        public static final double kaVoltSecondsSquaredPerMeter = 0.5;
        public static final double kvVoltSecondsPerMeter = 0.5;

        public static final double ksVolts = 0.5;
        public static final double kgVolts = 0.5;

        // Gyro
        public static final boolean GYRO_REVERSED = true;

        // Differential Drive Kinematics
        public static final double TRACK_WIDTH_METERS = 0.5334;// 0.6731; //0.7894023727640018 from characterization
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH_METERS);

        // Max speeds in m/s
        public static final double LOW_GEAR_MAX_SPEED = 2.5;
        public static final double HIGH_GEAR_MAX_SPEED = 0.5;

        // Max angular velocity in degrees per second
        public static final double MAX_ANGULAR_VELOCITY = 360;

        // Curvature drive
        public static final double QUICK_STOP_THRESHOLD = 0.2;
        public static final double QUICK_STOP_ALPHA = 0.1;

        public static final double DEADBAND = 0.05;
        public static final double CUBIC_WEIGHT = 0.4;
        public static final double CURVATURE_TURN_SCALAR = 0.65;

        public static final boolean VELOCITY_DRIVE_DEFAULT = false;
        public static final boolean SQUARE_INPUTS_DEFAULT = true;
        public static final DriveTypes DRIVE_TYPE_DEFAULT = DriveTypes.CURVATURE;
        public static final double QUICK_TURN_THRESHOLD = 0.1;

        public static final double ANGLE_TOLERANCE = 2.0;

        // PID Values
        public static final double kP = 2.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double ARCADE_STATIC_GAIN = 0.0;
        public static final double ARCADE_VELOCITY_GAIN = 0.0;
        // ?????? what even is this for
        // FF = kS * (sign of velocity) + kV * velocity + kA * acceleration (0)
        // so in theory keeping the setpoint 1.0 means that the feedforward will just be
        // the static voltage
        // might not need a ff object
        public static final double TURN_VELOCITY_SETPOINT = 1.0;

        // Slew Rate Limit
        public static final double SLEW_RATE_LIMIT = 1.0;

        public static final double LEFT_POWER_SCALE = 1;
        public static final double RIGHT_POWER_SCALE = 1;

        public static final double TURN_P = 0.03;
        public static final double TURN_I = 0.0;
        public static final double TURN_D = 0.0;

        // degrees
        public static final double TURN_TOLERANCE = 5.0;
        // degrees per second
        public static final double TURN_SPEED = 100.0;
        public static final double TURN_SPEED_TOLERANCE = 10;
        // degrees per second^2
        public static final double TURN_ACCEL = 100.0;

        // Pose estimation constants
        public static final Matrix<N5, N1> POSE_STATE_MEASUREMENT_STDEV = new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02,
                0.02, 0.01, 0.02, 0.02);
        public static final Matrix<N3, N1> POSE_LOCAL_MEASUREMENT_STDEV = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02,
                0.02, 0.01);
        public static final Matrix<N3, N1> POSE_GLOBAL_MEASUREMENT_STDEV = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1,
                0.1, 0.01);
    }

    // TODO: fill in values for gearbox
    public static final class GearboxConstants {
        public static enum PIDTypes {
            VELOCITY, MOTION
        };

        public static final double PID_P = 2.83e-6;// 0.0695; 2.83e-5; 2.3e-5
        public static final double PID_I = 0.0;
        public static final double PID_D = 0.0;

        public static final double VOLTAGE_COMP_SATURATION = 12.0;
        public static final boolean VOLTAGE_COMP_ENABLED = true;

        public static final double WHEEL_DIAMETER = 0.1524;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;// or .479?

        /**
         * Motor CPR / gear reduction = effective CPR
         **/
        public static final double HIGH_GEAR_REDUCTION = 8.33;// 15.0;
        public static final double HIGH_GEAR_EFFECTIVE_CPR = FALCON_CPR * HIGH_GEAR_REDUCTION;

        public static final double LOW_GEAR_REDUCTION = 3.67;
        public static final double LOW_GEAR_EFFECTIVE_CPR = FALCON_CPR * LOW_GEAR_REDUCTION;

        // Feed forward/back gains
        public static final double STATIC_GAIN = 0.625; // volts
        public static final double VELOCITY_GAIN = 4.7; // volt seconds per meter
        public static final double ACCEL_GAIN = 0.289; // volt seconds squared per meter

        public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(STATIC_GAIN, VELOCITY_GAIN,
                ACCEL_GAIN);

        // motion magic
        public static final double MOTION_P = 0.2;
        public static final double MOTION_I = 0.0;
        public static final double MOTION_D = 0.0;
        public static final double MOTION_F = 0.2;
        public static final double MOTION_CRUISE_VELOCITY = 1;
        public static final double MOTION_ACCELERATION = .5;
        public static final double MOTION_TOLERANCE = 200;

        public static final NeutralMode DEFAULT_NEUTRAL_MODE = NeutralMode.Brake;
    }

    public static final class IntakeConstants {
        public static final int TALON_ID = 5;
        public static final boolean TALON_INVERTED = true;

        public static final int SOLENOID_PCM_ID = 0;
        public static final PneumaticsModuleType SOLENOID_MODULE_TYPE = PneumaticsModuleType.CTREPCM;
        public static final int SOLENOID_FORWARD = 6;
        public static final int SOLENOID_REVERSE = 1;

        public static final double DEADBAND = DriveConstants.DEADBAND;
    }

    public static final class ShooterConstants {
        // Shooter
        public static final int SHOOTER_ID = 17;
        public static final boolean SHOOTER_INVERTED = true;

        public static final NeutralMode SHOOTER_NEUTRAL_MODE = NeutralMode.Coast;

        public static final double SHOOTER_GEAR_REDUCTION = 1.0;
        public static final double SHOOTER_EFFECTIVE_CPR = FALCON_CPR * SHOOTER_GEAR_REDUCTION;

        public static final double ACCELERATOR_GEAR_REDUCTION = 1.0;
        public static final double ACCELERATOR_EFFECTIVE_CPR = FALCON_CPR * ACCELERATOR_GEAR_REDUCTION;

        public static final int SHOOTER_MAX_RPM = 1000;
        public static final double SHOOTER_CLOSED_LOOP_RAMP = 0.0;

        public static final int ACCELERATOR_MAX_RPM = 0;
        public static final double ACCELERATOR_CLOSED_LOOP_RAMP = 0.0;

        // Shooter Velocity PIDF values
        public static final double SHOOTER_PID_P = 0.0;// 335;
        public static final double SHOOTER_PID_I = 0.0;
        public static final double SHOOTER_PID_D = 0.0;

        // Accelerator Velocity PIDF values
        public static final double ACCELERATOR_PID_P = 0.0;
        public static final double ACCELERATOR_PID_I = 0.0;
        public static final double ACCELERATOR_PID_D = 0.0;
        public static final int ENCODER_UNITS_PER_ROTATION = 4096;

        // Hood
        public static final int HOOD_ID = 1;
        public static final boolean HOOD_INVERTED = true;
        public static final NeutralMode HOOD_NEUTRAL_MODE = NeutralMode.Brake;
        public static final double Min_HOOD_PERCENTAGE_SPEED = 0.15;
        public static final double MAX_HOOD_PERCENTAGE_SPEED = 0.2;
        public static final double MAX_ANGLE = 30.0;

        // Hood Limit Switch
        public static final int HOOD_LIMIT_SWITCH_ID = 9; 

        // Goal Constants
        public static final double GOAL_HEIGHT = 2.5;

        public static final double DEFAULT_SPEED = 1;
        public static final double RPM_MAX_ERROR = 10;

        public static final double STATIC_GAIN = .334;
        public static final double VELOCITY_GAIN = .112;
        public static final double ACCEL_GAIN = .0115;

        public static final SimpleMotorFeedforward SHOOTER_FF = new SimpleMotorFeedforward(STATIC_GAIN, VELOCITY_GAIN,
                ACCEL_GAIN);

        public static final double SHOOTER_VOLTAGE_COMP_SATURATION = 12.0;
        public static final boolean SHOOTER_VOLTAGE_COMP_ENABLED = true;

        public static final double ACCELERATOR_VOLTAGE_COMP_SATURATION = 12.0;
        public static final boolean ACCELERATOR_VOLTAGE_COMP_ENABLED = true;
    }

    public static final class ClimberConstants {
        public static final int LEFT_NEO_ID = 21;
        public static final int RIGHT_NEO_ID = 20;
        public static final boolean LEFT_INVERTED = false;
        public static final boolean RIGHT_INVERTED = true;

        public static final double DEADBAND = 0.05;

        // TODO: bro im just randomly putting numbers here
        public static final double P = 0.1;
        public static final double I = 1e-4;
        public static final double D = 1.0;
        public static final double FF = 0.0;
        public static final double IZONE = 0.0;
        public static final double MAX_VELOCITY = 10.0;
        public static final double MIN_VELOCITY = 0.0;
        public static final double MAX_ACCELERATION = 10.0;
        public static final double ALLOWED_ERROR = 5.0;
        public static final int TIMEOUT_MS = 5;
        
    }
}