package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/** Elevator subsystem with feed-forward and PID for position */
public class ElevatorPID extends SubsystemBase {
    public static final double kMaxSpeed = 0.2; // meters per second
    public static final double kMaxAcceleration = 0.1; // meters per second squared

    private static final double kWheelRadius = 0.03; // meters
    private static final int kEncoderResolution = 2048;
    private static final int kGearingRatio = 100;
        
    public static final double kP = 1.2832; // 1.2832 as of 020323
    public static final double kI = 0; 
    public static final double kD = 0; // 

    public static final double kS = 0.059082; // 0.059082
    public static final double kV = 58.393;
    public static final double kA = 1.7428;
    public static final double kG = 0.029112;

    private static final double kNativeUnitsPerRotation = kEncoderResolution * kGearingRatio;
    private static final double kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
    private static final double kMetersPerRotation = 2 * Math.PI * kWheelRadius;
    private static final double kRotationsPerMeter = 1 / kMetersPerRotation;

    private final WPI_TalonFX m_motor = new WPI_TalonFX(6);

    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAcceleration));
    private final PIDController m_velocityController = new PIDController(5, 0, 0);
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

    private boolean m_velocityControlEnabled = true;

    private double m_velocitySetpoint = 0;

    private double m_lastVelocitySetpoint = 0;
    private double m_lastTime = Timer.getFPGATimestamp();

    public ElevatorPID() {
        m_motor.setInverted(true); // invert motor output

        // reset elevator
        m_motor.set(ControlMode.PercentOutput, 0);
        setGoal(new TrapezoidProfile.State(0, 0));
        m_velocityControlEnabled = true;
        m_velocitySetpoint = 0;

        m_motor.getSensorCollection().setIntegratedSensorPosition(0, 30); // reset encoders
    }

    /*
    * Convert from TalonFX elevator position in meters to native units and vice versa
    */
    public double heightToNative(double heightUnits) {
        return heightUnits * kRotationsPerMeter * kNativeUnitsPerRotation;
    }

    public double nativeToHeight(double encoderUnits) {
        
        return encoderUnits * kRotationsPerNativeUnit * kMetersPerRotation;
    }

    public void setGoal(TrapezoidProfile.State goalState) {
        m_controller.setGoal(goalState);
    }

    public boolean atGoal() {
        return m_controller.atGoal();
    }

    public void setVelocitySetpoint(double velocitySetpoint) {
        m_velocitySetpoint = velocitySetpoint;
    }

    public void setToVelocityControlMode(boolean on) {
        m_velocityControlEnabled = on;
    }

    public double getCurrentHeight() {
        return -nativeToHeight(m_motor.getSensorCollection().getIntegratedSensorPosition());
    }

    public double getCurrentVelocity() {
        return nativeToHeight(m_motor.getSensorCollection().getIntegratedSensorVelocity() * 10); // motor velocity is in ticks per 100ms
    }

    public void zeroEncoder() {
        m_motor.getSensorCollection().setIntegratedSensorPosition(0, 30);
    }

    /*
    * Compute voltages using feedforward and pid
    */
    @Override
    public void periodic() {
        double velocitySetpoint = m_velocityControlEnabled ? m_velocitySetpoint : m_controller.getSetpoint().velocity;
        double accelerationSetpoint = m_velocityControlEnabled ? 0.0 : (velocitySetpoint - m_lastVelocitySetpoint) / (Timer.getFPGATimestamp() - m_lastTime);

        double feedforward = m_feedforward.calculate(velocitySetpoint, accelerationSetpoint);
        double pid = m_velocityControlEnabled ? m_velocityController.calculate(getCurrentVelocity(), velocitySetpoint) : m_controller.calculate(getCurrentHeight());

        System.out.println(getCurrentHeight());
        m_motor.setVoltage(feedforward + pid);

        m_lastVelocitySetpoint = velocitySetpoint;
        m_lastTime = Timer.getFPGATimestamp();
    }
}
