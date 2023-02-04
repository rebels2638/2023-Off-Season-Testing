package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
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

public class ArmPID extends SubsystemBase {
    public static final double kMaxSpeed = 0.2; // meters per second
    public static final double kMaxAcceleration = 0.1; // meters per second squared

    private static final double kWheelRadius = 0.03; // meters
    private static final int kEncoderResolution = 2048;
    private static final int kGearingRatio = 100;
        
    public static final double kP = 8.6473;
    public static final double kI = 0; 
    public static final double kD = 0; 

    public static final double kS = 0.057774;
    public static final double kV = 16.376;
    public static final double kA = 0.41226;
    public static final double kG = 0.029112; // inaccurate

    private static final double kNativeUnitsPerRotation = kEncoderResolution * kGearingRatio;
    private static final double kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
    private static final double kMetersPerRotation = 2 * Math.PI * kWheelRadius;
    private static final double kRotationsPerMeter = 1 / kMetersPerRotation;

    private final WPI_TalonFX m_motor = new WPI_TalonFX(5);

    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAcceleration));
    private final PIDController m_velocityController = new PIDController(5, 0, 0);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(kS, kG, kV, kA);

    private boolean m_velocityControlEnabled = true;

    private double m_velocitySetpoint = 0;

    private double m_lastVelocitySetpoint = 0;
    private double m_lastTime = Timer.getFPGATimestamp();
  
    private final double gearingRatio = 36.0;
    private final double radius = 3.75; // measure radius of the big gear

    public ArmPID() {
        // reset 
        m_motor.set(ControlMode.PercentOutput, 0);
        setGoal(0);
        m_velocityControlEnabled = true;
        m_velocitySetpoint = 0;

        m_motor.getSensorCollection().setIntegratedSensorPosition(0, 30); // reset encoders
    }

    /*
    * Convert from TalonFX elevator position in meters to native units and vice versa
    */
    public double nativeToHeight(double encoderUnits) {
        return ((encoderUnits/2048) / gearingRatio) * (2 * Math.PI * radius);
    }
  
    public double angleToHeight(double angle)
    {
        return radius * angle;
    }

    public void setGoal(double goalAngle) {
        TrapezoidProfile.State goalState = new TrapezoidProfile.State(angleToHeight(goalAngle), 0.0);
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
        return nativeToHeight(m_motor.getSensorCollection().getIntegratedSensorPosition());
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

        System.out.println("x " + (velocitySetpoint) + " zzz" + feedforward);
        m_motor.setVoltage(feedforward);

        m_lastVelocitySetpoint = velocitySetpoint;
        m_lastTime = Timer.getFPGATimestamp();
    }

    public void reset(){
        m_motor.getSensorCollection().setIntegratedSensorPosition(0, 30);
    }
}