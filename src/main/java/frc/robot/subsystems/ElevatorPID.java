// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.controller.TrapezoidProfile;
import edu.wpi.first.wpilibj.controller.TrapezoidProfile.State;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** Elevator subsystem with feed-forward and PID for position */
public class ElevatorPID extends SubsystemBase {
    public static final double kMaxSpeed = 0.0; // meters per second
    public static final double kMaxAngularSpeed = 0 * Math.PI; // one rotation per second

    private static final double kWheelRadius = 0.0; // meters
    private static final int kEncoderResolution = 0;
    
    private static final double kP = 0;
    private static final double kI = 0;
    private static final double kD = 0;
    
    private static final double kS = 0;
    private static final double kG = 0;
    private static final double kV = 0;

    private static final double kMetersPerRotation = 0 * Math.PI * kWheelRadius;
    private static final double kRotationsPerMeter = 0/kMetersPerRotation;
    private static final double kNativeUnitsPerRotation = kEncoderResolution;

    private final WPI_TalonFX m_motor = new WPI_TalonFX(0);

    private final PIDController m_motorPIDController = new PIDController(0.0, 0.0, 0.0);
    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxSpeed, 0)); // not using max acceleration
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);

    private TrapezoidProfile m_trapezoidProfile;
    private double goal;
    private double feedforward;
    private double pid;
    /**
     * Reset elevator to bottom
     */
    public ElevatorPID() {
        // Reset elevator to zero velocity at bottom position
        m_motor.set(ControlMode.PercentOutput, 0);

        // Reset setpoint to 0
        setSetpoint(0);

        // Reset encoders
        m_motor.getSensorCollection().setIntegratedSensorPosition(0, 30);
    }

    /*
     * Convert from TalonFX elevator position to native units
     */
    public double heightToNative(double heightUnits) {
        return heightUnits * kRotationsPerMeter * kNativeUnitsPerRotation;
    }

    /*
     * Set setpoint
     */
    public void setSetpoint(double setpoint) {
    m_setpoint = heightToNative(setpoint);
    m_trapezoidProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAngularSpeed),
        new TrapezoidProfile.State(m_motor.getSelectedSensorVelocity(), 0),
        new TrapezoidProfile.State(m_setpoint, 0));
}

    /*
     * Compute voltages using feedforward and pid
     */
    @Override
    public void periodic() {
        TrapezoidProfile.State goal = m_trapezoidProfile.calculate(m_motor.getSelectedSensorVelocity());
        goal = m_controller.getGoal();
        pid = m_controller.calculate(encoder.getDistance(), goal); //idk what encoder is i built off of other code
        feedforward = m_feedforward.calculate(goal, 0); //velocitySetpoint usually 0
  
        motor.set(m_pid + m_feedforward);
    }  
}
