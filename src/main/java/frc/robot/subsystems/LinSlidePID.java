package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RebelUtil;
import frc.robot.Robot;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** Elevator subsystem with feed-forward and PID for position */
public class LinSlidePID extends SubsystemBase {
    public static final double kMaxSpeed = 0.1; // meters per second
    public static final double kMaxAcceleration = 0.1; // meters per second squared

    private static final double kWheelRadius = 0.0; // meters
    private static final int kEncoderResolution = 2048; // ??
    private static final int kGearingRatio = 0; // ??
        
    public static final double kP = 0;
    public static final double kI = 0; 
    public static final double kD = 0; 

    public static final double kS = 0.0; 
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kG = 0.0;

    private static final double kNativeUnitsPerRotation = kEncoderResolution * kGearingRatio * 1.32; // what is this constant
    private static final double kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
    private static final double kMetersPerRotation = 2 * Math.PI * kWheelRadius;
    private static final double kRotationsPerMeter = 1 / kMetersPerRotation;

    
    private final WPI_TalonFX m_motor = new WPI_TalonFX(0);

    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAcceleration));
    private final PIDController m_velocityController = new PIDController(10, 0, 0);
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

    public boolean m_velocityControlEnabled = true;

    private double m_velocitySetpoint = 0;
    private double m_voltageSetpoint = 0;
    private double m_heightAccumulator = 0;

    private double m_lastVelocitySetpoint = 0;
    private double m_lastVelocity = 0;
    private double m_lastTime = Timer.getFPGATimestamp();

    private static double kUpperLimit = 0.0; // meters
    private static double kLowerLimit = -0.0; // meters

    private final ShuffleboardTab tab;

    private final GenericEntry linSlideEncoderPosition;
    private final GenericEntry linSlidePosition;
    private final GenericEntry linSlideVelocity;
    private final GenericEntry linSlideAcceleration;
    private final GenericEntry linSlidePositionSetpoint;
    private final GenericEntry linSlideVelocitySetpoint;
    private final GenericEntry linSlideAccelerationSetpoint;
    private final GenericEntry voltageSupplied;
    private final GenericEntry voltageSetpoint;

    public LinSlidePID() {
        // m_motor.setInverted(true); // invert motor output

        // reset linSlide
        m_motor.set(ControlMode.PercentOutput, 0);
        setToVelocityControlMode(true);
        setVelocitySetpoint(0);
        setGoal(new TrapezoidProfile.State(0, 0));
        resetHeightAccumulator();
        m_controller.setTolerance(0.01, 0.05);

        tab = Shuffleboard.getTab("Elevator");
        linSlideEncoderPosition = tab.add("Lin_Encoder_Position", 0.0).getEntry();
        linSlidePosition = tab.add("Lin_Height", 0.0).getEntry();
        linSlideVelocity = tab.add("Lin_Velocity", 0.0).getEntry();
        linSlideAcceleration = tab.add("Lin_Acceleration", 0.0).getEntry();
        linSlidePositionSetpoint = tab.add("Lin_Height Setpoint", 0.0).getEntry();
        linSlideVelocitySetpoint = tab.add("Lin_Velocity Setpoint", 0.0).getEntry();
        linSlideAccelerationSetpoint = tab.add("Lin_Acceleration Setpoint", 0.0).getEntry();
        voltageSupplied = tab.add("Lin_Motor Voltage", 0.0).getEntry();
        voltageSetpoint = tab.add("Lin_Voltage Setpoint", 0.0).getEntry();

        tab.add("Lin_Zero Encoder",
                new InstantCommand(() -> zeroEncoder()));
    }

    /*
    * Convert from TalonFX linSlide position in meters to native units and vice versa
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
        resetHeightAccumulator();
    }

    public void resetHeightAccumulator() {
        m_heightAccumulator = getCurrentHeight();
    }

    public double getCurrentEncoderPosition() {
        return -m_motor.getSensorCollection().getIntegratedSensorPosition();
    }

    public double getCurrentEncoderRate() {
        return -m_motor.getSensorCollection().getIntegratedSensorVelocity() * 10; // motor velocity is in ticks per 100ms
    }

    public double getCurrentHeight() {
        return nativeToHeight(getCurrentEncoderPosition());
    }

    public double getCurrentVelocity() {
        return nativeToHeight(getCurrentEncoderRate());
    }

    public double getCurrentAcceleration() {
        return (getCurrentVelocity() - m_lastVelocity) / (Timer.getFPGATimestamp() - m_lastTime);
    }

    public double getHeightSetpoint() {
        return m_velocityControlEnabled ? m_heightAccumulator : m_controller.getSetpoint().position;
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
        linSlideEncoderPosition.setDouble(getCurrentEncoderPosition());
        linSlidePosition.setDouble(getCurrentHeight());
        linSlideVelocity.setDouble(getCurrentVelocity());
        linSlideAcceleration.setDouble(getCurrentAcceleration());
        linSlidePositionSetpoint.setDouble(getHeightSetpoint());
        linSlideVelocitySetpoint.setDouble(getVelocitySetpoint());
        linSlideAccelerationSetpoint.setDouble(getAccelerationSetpoint());
        voltageSupplied.setDouble(m_motor.getMotorOutputVoltage());
        voltageSetpoint.setDouble(m_voltageSetpoint);
    }

    /*
    * Compute voltages using feedforward and pid
    */
    @Override
    public void periodic() {
        double feedforward = m_feedforward.calculate(getVelocitySetpoint(), getAccelerationSetpoint());
        // if(getCurrentHeight() <= kSwitchToNoForcePoint && getCurrentVelocity() <= kSwitchToNoForceVelocity)
        double positionPID = m_controller.calculate(getCurrentHeight());
        double velocityPID = m_velocityController.calculate(getCurrentVelocity(), getVelocitySetpoint());
        double pid = m_velocityControlEnabled ? velocityPID : positionPID;

        double voltage = RebelUtil.constrain(feedforward + pid, -12.0, 12.0);
        if (getCurrentHeight() >= kUpperLimit && voltage > 0.0) {
            voltage = 0.0;
        } else if (getCurrentHeight() <= kLowerLimit && voltage < 0.0) {
            voltage = 0.0;
        }

        m_voltageSetpoint = voltage;
        System.out.println(voltage);
        // m_motor1.setVoltage(voltage);
        // m_motor2.setVoltage(voltage);

        updateShuffleboard();

        m_lastVelocitySetpoint = getVelocitySetpoint();
        m_lastVelocity = getCurrentVelocity();
        m_lastTime = Timer.getFPGATimestamp();
        m_heightAccumulator += getVelocitySetpoint() * Robot.kDefaultPeriod;
    }

    public void breakMotor() {
        m_motor.stopMotor();
    }
}
