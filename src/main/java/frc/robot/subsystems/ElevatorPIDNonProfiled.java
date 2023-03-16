package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RebelUtil;
import frc.robot.Robot;
import frc.robot.utils.ConstantsArmElevator.ElevatorConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/** Elevator subsystem with feed-forward and PID for position */
public class ElevatorPIDNonProfiled extends SubsystemBase {
    private static ElevatorPIDNonProfiled instance = null;

    public static final double kMaxSpeed = 0.5; // meters per second
    public static final double kMaxAcceleration = 0.1; // meters per second squared

    private static final double kWheelRadius = 0.018191; // meters
    private static final int kEncoderResolution = 2048; 
    private static final int kGearingRatio = 6;

    private static final double kNativeUnitsPerRotation = kEncoderResolution * kGearingRatio;
    private static final double kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
    private static  final double kMetersPerRotation = 2 * Math.PI * kWheelRadius;
    private static final double kRotationsPerMeter = 1 / kMetersPerRotation;

    
    private final WPI_TalonFX m_motor1 = new WPI_TalonFX(0);
    private final WPI_TalonFX m_motor2 = new WPI_TalonFX(3);

    private final PIDController m_controller = new PIDController(12, 0, 0);
    private final PIDController m_velocityController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    public boolean m_velocityControlEnabled = true;

    private double m_velocitySetpoint = 0;
    private double m_voltageSetpoint = 0;
    private double m_heightAccumulator = 0;

    private double m_lastVelocitySetpoint = 0;
    private double m_lastVelocity = 0;
    private double m_lastTime = Timer.getFPGATimestamp();

    private static double kUpperLimit = 0.843;
    private static double kLowerLimit = 0.01;

    private final ShuffleboardTab tab;

    private final GenericEntry elevatorEncoderPosition;
    private final GenericEntry elevatorPosition;
    private final GenericEntry elevatorVelocity;
    private final GenericEntry elevatorAcceleration;
    private final GenericEntry elevatorPositionSetpoint;
    private final GenericEntry elevatorVelocitySetpoint;
    private final GenericEntry elevatorAccelerationSetpoint;
    private final GenericEntry voltageSupplied;
    private final GenericEntry voltageSetpoint;

    public ElevatorPIDNonProfiled() {
        m_motor1.setInverted(false); // they changed the motor
        m_motor2.setInverted(false);

        // reset elevator
        m_motor1.setNeutralMode(NeutralMode.Brake);
        m_motor2.setNeutralMode(NeutralMode.Brake);
        zeroEncoder();

        m_motor1.set(ControlMode.PercentOutput, 0);
        m_motor2.set(ControlMode.PercentOutput, 0);
        setToVelocityControlMode(true);
        setGoal(0);
        setVelocitySetpoint(0);
        resetHeightAccumulator();
        m_controller.setTolerance(0.03, 0.03);
        
        tab = Shuffleboard.getTab("Elevator");
        elevatorEncoderPosition = tab.add("Encoder Position", 0.0).getEntry();
        elevatorPosition = tab.add("Height", 0.0).getEntry();
        elevatorVelocity = tab.add("Velocity", 0.0).getEntry();
        elevatorAcceleration = tab.add("Acceleration", 0.0).getEntry();
        elevatorPositionSetpoint = tab.add("Height Setpoint", 0.0).getEntry();
        elevatorVelocitySetpoint = tab.add("Velocity Setpoint", 0.0).getEntry();
        elevatorAccelerationSetpoint = tab.add("Acceleration Setpoint", 0.0).getEntry();
        voltageSupplied = tab.add("Motor Voltage", 0.0).getEntry();
        voltageSetpoint = tab.add("Voltage Setpoint", 0.0).getEntry();

        tab.add("Zero Encoder",
                new InstantCommand(() -> this.zeroEncoder()));
        zeroEncoder();
    }

    public static ElevatorPIDNonProfiled getInstance() {
        if (instance == null) {
            instance = new ElevatorPIDNonProfiled();
        }
        return instance;
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

    public void setGoal(double height) {
        m_controller.setSetpoint(height);
    }

    public boolean atGoal() {
        return m_controller.atSetpoint();
    }

    public void setVelocitySetpoint(double velocitySetpoint) {
        m_velocitySetpoint = velocitySetpoint;
    }

    public void setToVelocityControlMode(boolean on) {
        m_velocityControlEnabled = on;
        resetHeightAccumulator();
        this.setVelocitySetpoint(0);
    }

    public void resetHeightAccumulator() {
        m_heightAccumulator = getCurrentHeight();
    }

    public double getCurrentEncoderPosition() {
        return m_motor1.getSensorCollection().getIntegratedSensorPosition();
    }

    public double getCurrentEncoderRate() {
        return m_motor1.getSensorCollection().getIntegratedSensorVelocity() * 10; // motor velocity is in ticks per 100ms
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

    public void zeroEncoder() {
        m_motor1.getSensorCollection().setIntegratedSensorPosition(0, 30);
        m_motor2.getSensorCollection().setIntegratedSensorPosition(0, 30);
    }

    public void updateShuffleboard() {
        elevatorEncoderPosition.setDouble(getCurrentEncoderPosition());
        elevatorPosition.setDouble(getCurrentHeight());
        elevatorVelocity.setDouble(getCurrentVelocity());
        elevatorAcceleration.setDouble(getCurrentAcceleration());
        voltageSupplied.setDouble(m_motor1.getMotorOutputVoltage());
        voltageSetpoint.setDouble(m_voltageSetpoint);
    }

    /*
    * Compute voltages using feedforward and pid
    */
    @Override
    public void periodic() {
        // double m_error = m_controller.getSetpoint() - getCurrentHeight();
        double velocityPID = m_velocitySetpoint * 5;
        double positionPID = m_controller.calculate(getCurrentHeight());
        double pid = m_velocityControlEnabled ? velocityPID : positionPID;
        double feedforward = ElevatorConstants.kG + (pid == 0 ? 0 : pid < 0 ? -1 : 1) * ElevatorConstants.kS;
        double voltage = RebelUtil.constrain(feedforward + pid, -12.0, 12.0);
        if (getCurrentEncoderPosition() >= kUpperLimit && voltage >= ElevatorConstants.kG) {
            voltage = 0.0;
            System.out.println("STOPEDDD");

        } else if (getCurrentEncoderPosition() <= kLowerLimit && voltage < 0.0) {
            voltage = 0.0;
            System.out.println("STOPEDDD");
        }

        // System.out.println("Elevator : " + voltage);
        m_voltageSetpoint = RebelUtil.constrain(voltage,-12, 12);
        m_motor1.setVoltage(m_voltageSetpoint);
        m_motor2.setVoltage(m_voltageSetpoint);

        updateShuffleboard();
        m_lastVelocity = getCurrentVelocity();
        m_lastTime = Timer.getFPGATimestamp();
    }

    public void breakMotor() {
        m_motor1.stopMotor();
        m_motor2.stopMotor();
    }
}
