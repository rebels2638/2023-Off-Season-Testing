package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RebelUtil;
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
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class LinearSlidePID extends SubsystemBase {
    public static final double kMaxSpeed = 0.2; // meters per second
    public static final double kMaxAcceleration = 0.1; // meters per second squared

    private static final double kWheelRadius = 0.051; // meters
    private static final int kEncoderResolution = 4*4096;
    private static final int kGearingRatio = 0;
        
    public static final double kP = 0;
    public static final double kI = 0; 
    public static final double kD = 0;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;

    private static final double kNativeUnitsPerRotation = kEncoderResolution * kGearingRatio * 1.32; // what is this constant huh
    private static final double kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
    private static final double kMetersPerRotation = 2 * Math.PI * kWheelRadius;
    private static final double kRotationsPerMeter = 1 / kMetersPerRotation;

    private final WPI_TalonSRX m_motor = new WPI_TalonSRX(0); // idk what this is

    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAcceleration));
    private final PIDController m_velocityController = new PIDController(kP, kI, kD);
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

    public boolean m_velocityControlEnabled = true;

    private double m_velocitySetpoint = 0;

    private double m_lastVelocitySetpoint = 0;
    private double m_lastTime = Timer.getFPGATimestamp();

    private static double kUpperLimit = 0;
    private static double kLowerLimit = 0;

    public LinearSlidePID() {
        // m_motor.setInverted(true); // invert motor output

        // reset elevator
        m_motor.set(ControlMode.PercentOutput, 0);
        setGoal(new TrapezoidProfile.State(0, 0));
        m_velocityControlEnabled = true;
        m_velocitySetpoint = 0;
        m_controller.setTolerance(0.01, 0.05);

        // to edit?
        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.peakCurrentLimit = 40; // the peak current, in amps
        config.peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms
        config.continuousCurrentLimit = 30; // the current to maintain if the peak limit is triggered
        m_motor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder

        // m_motor.setSelectedSensorPosition(0, 0, 30); // reset encoders
    }


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
        return -nativeToHeight(m_motor.getSelectedSensorPosition());
    }

    public double getCurrentVelocity() {
        return nativeToHeight(m_motor.getSelectedSensorVelocity() * 10); // motor velocity is in ticks per 100ms
    }

    public void zeroEncoder() {
        m_motor.setSelectedSensorPosition(0, 0, 30);
    }

    /*
    * Compute voltages using feedforward and pid
    */
    @Override
    public void periodic() {
        double velocitySetpoint = m_velocityControlEnabled ? m_velocitySetpoint : m_controller.getSetpoint().velocity;
        double accelerationSetpoint = m_velocityControlEnabled ? 0.0 : (velocitySetpoint - m_lastVelocitySetpoint) / (Timer.getFPGATimestamp() - m_lastTime);

        double feedforward = m_feedforward.calculate(velocitySetpoint, accelerationSetpoint);
        double positionPID = m_controller.calculate(getCurrentHeight());
        double velocityPID = m_velocityController.calculate(getCurrentVelocity(), velocitySetpoint);
        double pid = m_velocityControlEnabled ? velocityPID : positionPID;

        double currentEncoder = m_motor.getSelectedSensorPosition();
        double voltage = RebelUtil.constrain(feedforward + pid, -12.0, 12.0);
        if (currentEncoder >= kUpperLimit && voltage > 0.0) {
            feedforward = 0.0;
        } else if (currentEncoder <= kLowerLimit && voltage < 0.0) {
            feedforward = 0.0;
        }

        System.out.println(voltage);
        // m_motor.setVoltage(voltage);

        m_lastVelocitySetpoint = velocitySetpoint;
        m_lastTime = Timer.getFPGATimestamp();
    }

    public void breakMotor() {
        m_motor.stopMotor();
    }
}
