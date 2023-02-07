package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RebelUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class ArmPID extends SubsystemBase {
    public static final double kMaxSpeed = 2.5; // radians? per second
    public static final double kMaxAcceleration = 1; // radians? per second squared

    private static final int kEncoderResolution = 2048;
    private static final int kGearingRatio = 81 * (64 / 20);
    private static final double kNativeUnitsPerRotation = kEncoderResolution * kGearingRatio;
    private static final double kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
    private static final double kRadiansPerRotation = 2 * Math.PI;
    private static final double kFeedforwardAngleOffset = 0.2666;
        
    public static final double kP = 8; // 5.8146
    public static final double kI = 0; 
    public static final double kD = 0.02;

    public static final double kS = 0.068689;
    public static final double kV = 4.3647;
    public static final double kA = 0.12205;
    public static final double kG = 0.051839;

    private final WPI_TalonFX m_motor = new WPI_TalonFX(5);

    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAcceleration));
    private final PIDController m_velocityController = new PIDController(0.004982, 0, 0);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(kS, kG, kV, kA);

    public boolean m_velocityControlEnabled = false;

    private double m_velocitySetpoint = 0;

    private double m_lastVelocitySetpoint = 0;
    private double m_lastTime = Timer.getFPGATimestamp();
    
    private static double kUpperLimit = 55000.0;
    private static double kLowerLimit = -65000.0;

    private final GenericEntry tab;
  
    public ArmPID() {
        // reset 
        m_motor.set(ControlMode.PercentOutput, 0);
        m_controller.setTolerance(0.01, 0.1);
        setGoal(0);
        setToVelocityControlMode(false);
        m_velocitySetpoint = 0;
        tab = Shuffleboard.getTab("SmartDashboard").add("Arm Angle", 0.0).getEntry();
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
    }

    public double getCurrentAngle() {
        return nativeToRad(m_motor.getSensorCollection().getIntegratedSensorPosition());
    }

    public double getCurrentVelocity() {
        return nativeToRad(m_motor.getSensorCollection().getIntegratedSensorVelocity() * 10); // motor velocity is in ticks per 100ms
    }

    public void zeroEncoder() {
        m_motor.getSensorCollection().setIntegratedSensorPosition(0, 30);
    }

    /*
    * Compute voltages using feedforward and pid
    */
    @Override
    public void periodic() {
        double angle = getCurrentAngle();
        tab.setDouble(angle);
        double velocitySetpoint = m_velocityControlEnabled ? m_velocitySetpoint : m_controller.getSetpoint().velocity;
        double positionSetpoint = m_velocityControlEnabled ? angle + velocitySetpoint * 0.02 : m_controller.getSetpoint().position;
        double accelerationSetpoint = m_velocityControlEnabled ? 0.0 : (velocitySetpoint - m_lastVelocitySetpoint) / (Timer.getFPGATimestamp() - m_lastTime);

        double feedforward = m_feedforward.calculate(positionSetpoint + kFeedforwardAngleOffset, velocitySetpoint, accelerationSetpoint);
        double positionPID = m_controller.calculate(angle);
        double velocityPID = m_velocityController.calculate(getCurrentVelocity(), velocitySetpoint);
        double pid = m_velocityControlEnabled ? velocityPID : positionPID;

        double currentEncoder = m_motor.getSensorCollection().getIntegratedSensorPosition();
        double voltage = RebelUtil.constrain(feedforward + pid, -12.0, 12.0);
        if (currentEncoder >= kUpperLimit && voltage > 0.0) {
            feedforward = 0.0;
        } else if (currentEncoder <= kLowerLimit && voltage < 0.0) {
            feedforward = 0.0;
        }
        
        m_motor.setVoltage(voltage);

        m_lastVelocitySetpoint = velocitySetpoint;
        m_lastTime = Timer.getFPGATimestamp();
    }
}
