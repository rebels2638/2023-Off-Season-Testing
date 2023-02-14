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
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class FourBarArmPID extends SubsystemBase {
    public static final double kMaxSpeed = 2.5; // radians? per second
    public static final double kMaxAcceleration = 1; // radians? per second squared

    private static final int kEncoderResolution = 4096;
    private static final int kGearingRatio = 100 * (64 / 20);
    private static final double kNativeUnitsPerRotation = kEncoderResolution * kGearingRatio;
    private static final double kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
    private static final double kRadiansPerRotation = 2 * Math.PI;
    private static final double kFeedforwardAngleOffset = -1.5715;
        
    public static final double kP = 0; // 11.31 for velocity
    public static final double kI = 0; 
    public static final double kD = 0;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;

    private final WPI_TalonSRX m_motor = new WPI_TalonSRX(7);

    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAcceleration));
    private final PIDController m_velocityController = new PIDController(kP, kI, kD);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(kS, kG, kV, kA);

    public boolean m_velocityControlEnabled = false;

    private double m_velocitySetpoint = 0;

    private double m_lastVelocitySetpoint = 0;
    private double m_lastTime = Timer.getFPGATimestamp();
    
    private static double kUpperLimit = 0;
    private static double kLowerLimit = 0;

    private final GenericEntry tab;
  
    public FourBarArmPID() {
        m_motor.setInverted(true); // i think?

        // reset 
        m_motor.set(ControlMode.PercentOutput, 0);
        m_controller.setTolerance(0.01, 0.1);
        setGoal(0);
        setToVelocityControlMode(false);
        m_velocitySetpoint = 0;
        tab = Shuffleboard.getTab("SmartDashboard").add("FourBar Angle", 0.0).getEntry();

        // to edit?
        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.peakCurrentLimit = 40; // the peak current, in amps
        config.peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms
        config.continuousCurrentLimit = 30; // the current to maintain if the peak limit is triggered
        m_motor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder

        // m_motor.setSelectedSensorPosition(0, 0, 30); // reset encoders
    }

    // convert from TalonSRX arm angle in native units to radians
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

    public double getCurrentMotorAngle() {
        return nativeToRad(m_motor.getSelectedSensorPosition());
    }

    public double getCurrentVelocity() {
        return nativeToRad(m_motor.getSelectedSensorVelocity() * 10); // motor velocity is in ticks per 100ms
    }

    public void zeroEncoder() {
        m_motor.setSelectedSensorPosition(0, 0, 30);
    }

    // The god formula
    public double motorAngleToSlideAngle() {
        return 0.0;
    }

    /*
    * Compute voltages using feedforward and pid
    */
    @Override
    public void periodic() {
        double angle = getCurrentMotorAngle();
        tab.setDouble(angle);
        double velocitySetpoint = m_velocityControlEnabled ? m_velocitySetpoint : m_controller.getSetpoint().velocity;
        double positionSetpoint = m_velocityControlEnabled ? angle + velocitySetpoint * 0.02 : m_controller.getSetpoint().position;
        double accelerationSetpoint = m_velocityControlEnabled ? 0.0 : (velocitySetpoint - m_lastVelocitySetpoint) / (Timer.getFPGATimestamp() - m_lastTime);

        double feedforward = m_feedforward.calculate(positionSetpoint + kFeedforwardAngleOffset, velocitySetpoint, accelerationSetpoint);
        double positionPID = m_controller.calculate(angle);
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

        m_lastVelocitySetpoint = velocitySetpoint;
        m_lastTime = Timer.getFPGATimestamp();
        m_motor.setVoltage(voltage);
    }
}
