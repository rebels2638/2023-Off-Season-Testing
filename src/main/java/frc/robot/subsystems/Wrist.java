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
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Wrist extends SubsystemBase {
    
    public static final double kMaxSpeed = 2.5; // radians? per second
    public static final double kMaxAcceleration = 1; // radians? per second squared

    private static final int kEncoderResolution = 2048;
    private static final double kGearingRatio = 90.0 * (30.0 / 12.0);
    private static final double kNativeUnitsPerRotation = kEncoderResolution * kGearingRatio;
    private static final double kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
    private static final double kRadiansPerRotation = 2 * Math.PI;
    private static final double kFeedforwardAngleOffset = 0;

    public static final double kP = 3; // 5.8146 (it was actually 7.1682 but we increased it)
    public static final double kI = 0;
    public static final double kD = 0.0;

    public static final double kS = 0.10299; // 0.068689;
    public static final double kV = 0.36865; // 4.3647;
    public static final double kA = 0.019917; // 0.12205;
    public static final double kG = 0.18409; // 0.051839;

    private final WPI_TalonFX m_wrist = new WPI_TalonFX(5);
    
    private final PIDController m_controller = new PIDController(kP, kI, kD);
    private final PIDController m_velocityController = new PIDController(0.0087647, 0, 0);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(kS, kG, kV, kA);

    public boolean m_velocityControlEnabled = false;

    private double m_velocitySetpoint = 0;
    private double m_voltageSetpoint = 0;
    private double m_angleAccumulator = 0;

    private double m_lastVelocitySetpoint = 0;
    private double m_lastVelocity = 0;
    private double m_lastTime = Timer.getFPGATimestamp();

    private static double kUpperLimit = 110000.0;
    private static double kLowerLimit = -44900.0;

    private final ShuffleboardTab tab;


    private final GenericEntry wristEncoderPosition;
    private final GenericEntry wristPosition;
    private final GenericEntry wristVelocity;
    private final GenericEntry wristAcceleration;
    private final GenericEntry wristPositionSetpoint;
    private final GenericEntry wristVelocitySetpoint;
    private final GenericEntry wristAccelerationSetpoint;
    private final GenericEntry voltageSupplied;
    private final GenericEntry voltageSetpoint;

    public Wrist() {
        // reset
        m_wrist.set(ControlMode.PercentOutput, 0);
        m_controller.setTolerance(0.05, 0.1);
        setToVelocityControlMode(true);
        setVelocitySetpoint(0);
        setGoal(0);
        resetAngleAccumulator();
        
        // shuff
        tab = Shuffleboard.getTab("Arm");
        wristEncoderPosition = tab.add("Encoder Position", 0.0).getEntry();
        wristPosition = tab.add("Angle", 0.0).getEntry();
        wristVelocity = tab.add("Velocity", 0.0).getEntry();
        wristAcceleration = tab.add("Acceleration", 0.0).getEntry();
        wristPositionSetpoint = tab.add("Angle Setpoint", 0.0).getEntry();
        wristVelocitySetpoint = tab.add("Velocity Setpoint", 0.0).getEntry();
        wristAccelerationSetpoint = tab.add("Acceleration Setpoint", 0.0).getEntry();
        voltageSupplied = tab.add("Motor Voltage", 0.0).getEntry();
        voltageSetpoint = tab.add("Voltage Setpoint", 0.0).getEntry();

        tab.add("Zero Encoder",
                new InstantCommand(() -> this.zeroEncoder()));
        
    }
    
    public double nativeToRad(double encoderUnits) {
        return encoderUnits * kRotationsPerNativeUnit * kRadiansPerRotation;
    }

    public void setGoal(double goalAngle) {
        m_controller.setSetpoint(goalAngle + kFeedforwardAngleOffset); // radians
    }

    public boolean atGoal() {
        return m_controller.atSetpoint();
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
        return m_wrist.getSensorCollection().getIntegratedSensorPosition();
    }

    public double getCurrentEncoderRate() {
        return m_wrist.getSensorCollection().getIntegratedSensorVelocity() * 10; // motor velocity is in ticks per 100ms
    }

    public double getCurrentAngle() {
        return nativeToRad(getCurrentEncoderPosition()) + kFeedforwardAngleOffset;
    }

    public double getCurrentVelocity() {
        return nativeToRad(getCurrentEncoderRate());
    }

    public double getCurrentAcceleration() {
        return (getCurrentVelocity() - m_lastVelocity) / (Timer.getFPGATimestamp() - m_lastTime);
    }

    public double getAngleSetpoint() {
        return m_velocityControlEnabled ? m_angleAccumulator
                : m_controller.getSetpoint();
    }

    public double getVelocitySetpoint() {
        return m_velocityControlEnabled ? m_velocitySetpoint : 0;
    }

    public double getAccelerationSetpoint() {
        return m_velocityControlEnabled ? 0.0
                : (getVelocitySetpoint() - m_lastVelocitySetpoint) / (Timer.getFPGATimestamp() - m_lastTime);
    }

    public void zeroEncoder() {
        m_wrist.getSensorCollection().setIntegratedSensorPosition(0, 30);
    }

    public void updateShuffleboard() {
        wristEncoderPosition.setDouble(getCurrentEncoderPosition());
        wristPosition.setDouble(getCurrentAngle());
        wristVelocity.setDouble(getCurrentVelocity());
        wristAcceleration.setDouble(getCurrentAcceleration());
        wristPositionSetpoint.setDouble(getAngleSetpoint());
        wristVelocitySetpoint.setDouble(getVelocitySetpoint());
        wristAccelerationSetpoint.setDouble(getAccelerationSetpoint());
        voltageSupplied.setDouble(m_wrist.getMotorOutputVoltage());
        voltageSetpoint.setDouble(m_voltageSetpoint);
    }

    /*
     * Compute voltages using feedforward and pid
     */
    @Override
    public void periodic() {
        double positionPID = m_controller.calculate(getCurrentAngle());
        double velocityPID = m_velocitySetpoint * 4;
        double pid = (m_velocityControlEnabled ? velocityPID : positionPID);
        // double feedforward = kG + (pid == 0 ? 0 : pid < 0 ? -1 : 1) * kS;
        // double velocityPID = m_velocityController.calculate(getCurrentVelocity(), getVelocitySetpoint());

        double voltage = RebelUtil.constrain(pid, -12.0, 12.0);
        if (getCurrentEncoderPosition() >= kUpperLimit && voltage > 0.0) {
            voltage = 0.0;
        } else if (getCurrentEncoderPosition() <= kLowerLimit && voltage < 0.0) {
            voltage = 0.0;
        }
        System.out.println(voltage);
        m_voltageSetpoint = voltage;
        RebelUtil.constrain(m_voltageSetpoint, -4, 4);

        m_wrist.setVoltage(voltage);

        updateShuffleboard();

        m_lastVelocitySetpoint = getVelocitySetpoint();
        m_lastVelocity = getCurrentVelocity();
        m_lastTime = Timer.getFPGATimestamp();
        m_angleAccumulator += getVelocitySetpoint() * Robot.kDefaultPeriod;
    }
}
