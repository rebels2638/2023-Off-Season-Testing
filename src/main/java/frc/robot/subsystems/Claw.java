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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Claw extends SubsystemBase {
    
    private static Claw instance = null;
    private final DoubleSolenoid solenoid;
    private boolean state; // push is true, and pull is false
    
    public static final double kMaxSpeed = 2.5; // radians? per second
    public static final double kMaxAcceleration = 1; // radians? per second squared

    private static final int kEncoderResolution = 2048;
    private static final int kGearingRatio = 81 * (64 / 20);
    private static final double kNativeUnitsPerRotation = kEncoderResolution * kGearingRatio;
    private static final double kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
    private static final double kRadiansPerRotation = 2 * Math.PI;
    private static final double kFeedforwardAngleOffset = -0.87997;

    public static final double kP = 8.1682; // 5.8146 (it was actually 7.1682 but we increased it)
    public static final double kI = 0.2;
    public static final double kD = 0;

    public static final double kS = 0.21098; // 0.068689;
    public static final double kV = 3.9629; // 4.3647;
    public static final double kA = 0.29063; // 0.12205;
    public static final double kG = 0.11064; // 0.051839;

    private final WPI_TalonFX m_wrist = new WPI_TalonFX(5);
    

    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD,
            new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAcceleration));
    private final PIDController m_velocityController = new PIDController(0.004982, 0, 0);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(kS, kG, kV, kA);

    public boolean m_velocityControlEnabled = false;

    private double m_velocitySetpoint = 0;
    private double m_voltageSetpoint = 0;
    private double m_angleAccumulator = 0;

    private double m_lastVelocitySetpoint = 0;
    private double m_lastVelocity = 0;
    private double m_lastTime = Timer.getFPGATimestamp();

    private static double kUpperLimit = 55000.0; // change
    private static double kLowerLimit = -55000.0; // change

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

    public Claw() {
        // solenoid stuff
        this.solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 4);
        this.push();
        state = true;
        
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
                new InstantCommand(() -> zeroEncoder()));
    }
    
    // Singleton class, call getInstance to access instead of the constructor.
    public static Claw getInstance() {
        if (instance == null) {
            instance = new Claw();
        }
        return instance;
    }

    public void push() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
        state = true;
    }

    public void pull() {
        solenoid.set(DoubleSolenoid.Value.kForward);
        state = false;
    }

    public void toggle() {
        if (state) {
            pull();
            return;
        } else {
            push();
            return;
        }
    }
    
    public double nativeToRad(double encoderUnits) {
        return encoderUnits * kRotationsPerNativeUnit * kRadiansPerRotation;
    }

    public void setGoal(double goalAngle) {
        m_controller.setGoal(goalAngle + kFeedforwardAngleOffset); // radians
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
        System.out.println(voltage);
        // m_wrist.setVoltage(voltage);

        updateShuffleboard();

        m_lastVelocitySetpoint = getVelocitySetpoint();
        m_lastVelocity = getCurrentVelocity();
        m_lastTime = Timer.getFPGATimestamp();
        m_angleAccumulator += getVelocitySetpoint() * Robot.kDefaultPeriod;
    }
}
