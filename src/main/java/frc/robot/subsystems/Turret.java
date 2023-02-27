package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RebelUtil;
import frc.robot.Robot;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Turret extends SubsystemBase {
    public static final double kMaxSpeed = 2.5; // radians per second
    public static final double kMaxAcceleration = 1; // radians per second squared

    private static final int kEncoderResolution = 2048;
    private static final int kGearingRatio = 81 * (64 / 20);
    private static final double kNativeUnitsPerRotation = kEncoderResolution * kGearingRatio;
    private static final double kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
    private static final double kRadiansPerRotation = 2 * Math.PI;

    public static final double kP = 8.1682;
    public static final double kI = 0.2;
    public static final double kD = 0;

    public static final double kS = 0.21098;
    public static final double kV = 3.9629;
    public static final double kA = 0.29063;

    private final WPI_TalonFX m_motor = new WPI_TalonFX(9);

    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD,
            new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAcceleration));
    private final PIDController m_velocityController = new PIDController(0.004982, 0, 0);
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    public boolean m_velocityControlEnabled = false;

    private double m_velocitySetpoint = 0;
    private double m_voltageSetpoint = 0;
    private double m_angleAccumulator = 0;

    private double m_lastVelocitySetpoint = 0;
    private double m_lastVelocity = 0;
    private double m_lastTime = Timer.getFPGATimestamp();

    private static double kUpperLimit = 55000.0;
    private static double kLowerLimit = -55000.0;

    private final ShuffleboardTab tab;

    private final GenericEntry encoderPosition;
    private final GenericEntry angle;
    private final GenericEntry velocity;
    private final GenericEntry acceleration;
    private final GenericEntry angleSetpoint;
    private final GenericEntry velocitySetpoint;
    private final GenericEntry accelerationSetpoint;
    private final GenericEntry voltageSupplied;
    private final GenericEntry voltageSetpoint;

    public Turret() {
        // reset
        m_motor.set(ControlMode.PercentOutput, 0);
        m_controller.setTolerance(0.05, 0.1);
        setToVelocityControlMode(true);
        setVelocitySetpoint(0);
        setGoal(0);
        resetAngleAccumulator();

        tab = Shuffleboard.getTab("Turret");
        encoderPosition = tab.add("Encoder Position", 0.0).getEntry();
        angle = tab.add("Angle", 0.0).getEntry();
        velocity = tab.add("Velocity", 0.0).getEntry();
        acceleration = tab.add("Acceleration", 0.0).getEntry();
        angleSetpoint = tab.add("Angle Setpoint", 0.0).getEntry();
        velocitySetpoint = tab.add("Velocity Setpoint", 0.0).getEntry();
        accelerationSetpoint = tab.add("Acceleration Setpoint", 0.0).getEntry();
        voltageSupplied = tab.add("Motor Voltage", 0.0).getEntry();
        voltageSetpoint = tab.add("Voltage Setpoint", 0.0).getEntry();

        tab.add("Zero Encoder",
                new InstantCommand(() -> zeroEncoder()));
    }

    /*
     * Convert from TalonFX arm angle in native units to radians
     */
    public double nativeToRad(double encoderUnits) {
        return encoderUnits * kRotationsPerNativeUnit * kRadiansPerRotation;
    }

    public void setGoal(double goalAngle) {
        m_controller.setGoal(goalAngle); // radians
    }

    public boolean atGoal() {
        return m_controller.atGoal();
    }

    public void setVelocitySetpoint(double velocitySetpoint) {
        m_velocitySetpoint = velocitySetpoint;
    }

    public void setToVelocityControlMode(boolean on) {
        m_velocityControlEnabled = on;
        resetAngleAccumulator();
    }

    public void resetAngleAccumulator() {
        m_angleAccumulator = getCurrentAngle();
    }

    public double getCurrentEncoderPosition() {
        return m_motor.getSensorCollection().getIntegratedSensorPosition();
    }

    public double getCurrentEncoderRate() {
        return m_motor.getSensorCollection().getIntegratedSensorVelocity() * 10; // motor velocity is in ticks per 100ms
    }

    public double getCurrentAngle() {
        return nativeToRad(getCurrentEncoderPosition());
    }

    public double getCurrentVelocity() {
        return nativeToRad(getCurrentEncoderRate());
    }

    public double getCurrentAcceleration() {
        return (getCurrentVelocity() - m_lastVelocity) / (Timer.getFPGATimestamp() - m_lastTime);
    }

    public double getAngleSetpoint() {
        return m_velocityControlEnabled ? m_angleAccumulator
                : m_controller.getSetpoint().position;
    }

    public double getVelocitySetpoint() {
        return m_velocityControlEnabled ? m_velocitySetpoint : m_controller.getSetpoint().velocity;
    }

    public double getAccelerationSetpoint() {
        return m_velocityControlEnabled ? 0.0
                : (getVelocitySetpoint() - m_lastVelocitySetpoint) / (Timer.getFPGATimestamp() - m_lastTime);
    }

    public void zeroEncoder() {
        m_motor.getSensorCollection().setIntegratedSensorPosition(0, 30);
    }

    public void updateShuffleboard() {
        encoderPosition.setDouble(getCurrentEncoderPosition());
        angle.setDouble(getCurrentAngle());
        velocity.setDouble(getCurrentVelocity());
        acceleration.setDouble(getCurrentAcceleration());
        angleSetpoint.setDouble(getAngleSetpoint());
        velocitySetpoint.setDouble(getVelocitySetpoint());
        accelerationSetpoint.setDouble(getAccelerationSetpoint());
        voltageSupplied.setDouble(m_motor.getMotorOutputVoltage());
        voltageSetpoint.setDouble(m_voltageSetpoint);
    }

    /*
     * Compute voltages using feedforward and pid
     */
    @Override
    public void periodic() {
        double feedforward = m_feedforward.calculate(getAngleSetpoint(), getVelocitySetpoint(),
                getAccelerationSetpoint());
        double positionPID = m_controller.calculate(getCurrentAngle());
        double velocityPID = m_velocityController.calculate(getCurrentVelocity(), getVelocitySetpoint());
        double pid = m_velocityControlEnabled ? velocityPID : positionPID;

        double voltage = RebelUtil.constrain(feedforward + pid, -12.0, 12.0);
        if (getCurrentEncoderPosition() >= kUpperLimit && voltage > 0.0) {
            voltage = 0.0;
        } else if (getCurrentEncoderPosition() <= kLowerLimit && voltage < 0.0) {
            voltage = 0.0;
        }

        m_voltageSetpoint = voltage;
        m_motor.setVoltage(voltage);

        updateShuffleboard();

        m_lastVelocitySetpoint = getVelocitySetpoint();
        m_lastVelocity = getCurrentVelocity();
        m_lastTime = Timer.getFPGATimestamp();
        m_angleAccumulator += getVelocitySetpoint() * Robot.kDefaultPeriod;
    }
}
