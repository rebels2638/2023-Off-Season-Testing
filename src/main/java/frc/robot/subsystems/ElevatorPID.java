// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.fasterxml.jackson.databind.deser.impl.TypeWrappedDeserializer;

/** Elevator subsystem with feed-forward and PID for position */
public class ElevatorPID extends SubsystemBase {
    public static final double kMaxSpeed = 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

    private static final double kWheelRadius = 0.0508; // meters
    private static final int kEncoderResolution = 4096;

    private static final double kMetersPerRotation = 2 * Math.PI * kWheelRadius;
    private static final double kRotationsPerMeter = 1/kMetersPerRotation;
    private static final double kNativeUnitsPerRotation = kEncoderResolution;

    private final WPI_TalonFX m_motor = new WPI_TalonFX(6);

    private final PIDController m_motorPIDController = new PIDController(Constants.ElevatorPIDConstants.kP, Constants.ElevatorPIDConstants.kI, Constants.ElevatorPIDConstants.kD);
    
    // Gains are for example purposes only - must be determined for your own robot!
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0, 0, 0, 0);
    private TrapezoidProfile m_trapezoidProfile;
    private double m_setpoint;

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
    }

    /*
     * Compute voltages using feedforward and pid
     */
    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // control for constant position
        //double ff = m_feedforward.calculate(m_setpoint, 0.0);
        //double pid = m_motorPIDController.calculate(m_motor.getSelectedSensorVelocity(), m_setpoint);
        TrapeZoidProfile.State goal = m_trapezoidProfile.calculate(m_motor.getSelectedSensorePosition());
        // set voltage
        m_motor.setVoltage(pid);
    }
}
