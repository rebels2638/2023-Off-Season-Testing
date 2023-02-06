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
    public static final double kMaxSpeed = 0.5; // radians? per second
    public static final double kMaxAcceleration = 0.2; // radians? per second squared

    private static final double kWheelRadius = 0.0375; // meters
    private static final int kEncoderResolution = 2048;
    private final double kGearingRatio = 36.0;
        
    public static final double kP = 0.0015084; // 5.8146 for position loop
    public static final double kI = 0; 
    public static final double kD = 0; // 0.55603 for position loop

    public static final double kS = 0.056031;
    public static final double kV = 0.60777;
    public static final double kA = 0.02306;
    public static final double kG = 0.042845;

    private final WPI_TalonFX m_motor = new WPI_TalonFX(5);

    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAcceleration));
    private final PIDController m_velocityController = new PIDController(5, 0, 0);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(kS, kG, kV, kA);

    private boolean m_velocityControlEnabled = true;

    private double m_velocitySetpoint = 0;

    private double m_lastVelocitySetpoint = 0;
    private double m_lastTime = Timer.getFPGATimestamp();
    
    private static double kUpperLimit = 55000.0;
    private static double kLowerLimit = -65000.0;
  
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
    public double nativeToAngle(double encoderUnits) {
        return ((encoderUnits/kEncoderResolution) / kGearingRatio) * (2 * Math.PI);
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
        return nativeToAngle(m_motor.getSensorCollection().getIntegratedSensorPosition());
    }

    public double getCurrentVelocity() {
        return nativeToAngle(m_motor.getSensorCollection().getIntegratedSensorVelocity() * 10); // motor velocity is in ticks per 100ms
    }

    public void zeroEncoder() {
        m_motor.getSensorCollection().setIntegratedSensorPosition(0, 30);
    }

    /*
    * Compute voltages using feedforward and pid
    */
    @Override
    public void periodic() {
        double velocitySetpoint = m_velocityControlEnabled ? m_velocitySetpoint : m_controller.getSetpoint().position;
        double accelerationSetpoint = m_velocityControlEnabled ? 0.0 : (velocitySetpoint - m_lastVelocitySetpoint) / (Timer.getFPGATimestamp() - m_lastTime);

        double feedforward = m_feedforward.calculate(m_controller.getSetpoint().position, m_controller.getSetpoint().velocity);
        double positionPID = m_controller.calculate(getCurrentAngle());
        double velocityPID = m_velocityController.calculate(getCurrentVelocity(), velocitySetpoint);
        double pid = m_velocityControlEnabled ? velocityPID : positionPID;

        double currentEncoder = m_motor.getSensorCollection().getIntegratedSensorPosition();
        if (currentEncoder >= kUpperLimit && feedforward > 0.0) {
            feedforward = 0.0;
        }
        else if (currentEncoder <= kLowerLimit && feedforward < 0.0) {
            feedforward = 0;
        }

        System.out.println("current position: " + getCurrentAngle() + "        feedforward: " + feedforward);
        m_motor.setVoltage(Math.max(-3, Math.min(3, feedforward)));

        m_lastVelocitySetpoint = velocitySetpoint;
        m_lastTime = Timer.getFPGATimestamp();
    }

    public void reset(){
        m_motor.getSensorCollection().setIntegratedSensorPosition(0, 30);
    }
}
