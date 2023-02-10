package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.motorcontrol.can.WPI_VicptorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; 
import edu.wpi.first.math.controller.PIDController;
import frc.lib.RebelUtil;

import edu.wpi.first.math.controller.ProfiledPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class FourBarArm extends SubsystemBase {
    private final WPI_TalonSRX talon;
    private final WPI_TalonSRX motor_775;

    public static final double kMaxSpeed = 2.5; // radians? per second
    public static final double kMaxAcceleration = 1; // radians? per second squared

    private static final int kEncoderResolution = 4096;
    private static final int kGearingRatio = 100 * (66 / 18);
    private static final double kNativeUnitsPerRotation = kEncoderResolution * kGearingRatio;
    private static final double kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
    private static final double kRadiansPerRotation = 2 * Math.PI;
    private static final double kFeedforwardAngleOffset = -0.87997;
        
    public static final double kP = 8.1682; // 5.8146 (it was actually 7.1682 but we increased it)
    public static final double kI = 0.2; 
    public static final double kD = 0;

    public static final double kS = 0.21098; //0.068689;
    public static final double kV = 3.9629; //4.3647;
    public static final double kA = 0.29063; //0.12205;
    public static final double kG = 0.11064; //0.051839;

    
    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAcceleration));
    private final PIDController m_velocityController = new PIDController(0.004982, 0, 0);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(kS, kG, kV, kA);

    public boolean m_velocityControlEnabled = false;

    private double m_velocitySetpoint = 0;

    private double m_lastVelocitySetpoint = 0;
    private double m_lastTime = Timer.getFPGATimestamp();
    
    private static double kUpperLimit = 55000.0;
    private static double kLowerLimit = -55000.0;

    //private final WPI_VictorSPX victor;
    // private final CANSparkMax spark;
    
    private static FourBarArm instance = null;

    public FourBarArm() {
        //this.victor = new WPI_VictorSPX(5);
        this.talon = new WPI_TalonSRX(7); // one instance of TalonSRX, replaced IntakeConstants.TALON_ID
        // this.spark = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        talon.setNeutralMode(NeutralMode.Brake);
        this.motor_775 = new WPI_TalonSRX(8);
        this.talon.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative,0,30);
        motor_775.setNeutralMode(NeutralMode.Brake);
        // spark.setInverted(true);
        m_controller.setTolerance(0.05, 0.1);
        setGoal(new TrapezoidProfile.State(0, 0));
        setToVelocityControlMode(false);
        m_velocitySetpoint = 0;

    }

    public void setGoal(TrapezoidProfile.State goalState) {
        m_controller.setGoal(goalState); // radians
    }

    public void setVelocitySetpoint(double velocitySetpoint) {
        m_velocitySetpoint = velocitySetpoint;
    }

    public void setToVelocityControlMode(boolean on) {
        m_velocityControlEnabled = on;
    }

    public double getCurrentAngle() {
        return nativeToRad(talon.getSelectedSensorPosition()) + kFeedforwardAngleOffset;
    }

    public boolean atGoal() {
        return m_controller.atGoal();
    }

    public double getCurrentVelocity() {
        return nativeToRad(talon.getSelectedSensorVelocity() * 10); // motor velocity is in ticks per 100ms
    }

    public void zeroEncoder() {
        talon.setSelectedSensorPosition(0,0,30);
    }

    public double nativeToRad(double encoderUnits) {
        return encoderUnits * kRotationsPerNativeUnit * kRadiansPerRotation;
    }


    // Singleton class, call getInstance to access instead of the constructor.
    public static FourBarArm getInstance() {
        if (instance == null) {
            instance = new FourBarArm();
        }
        return instance;
    }

    public void setPercentOutputTalon(double percent) {

        talon.set(ControlMode.PercentOutput, percent); // set talon speed based on input from XboxController.getleftY(), ie the input range on left y should map to the speed???? where speed is in range -1,1 and the xbox controller left joy stick is also -1,1???
    }

    public void setPercentOutput775(double percent) {

        motor_775.set(ControlMode.PercentOutput, -percent);
    }
    
    // public void setSpeedSpark(double speed) {
        
    //     spark.set(speed);
        
    // }

    // @Override
    // public void periodic() {

    //     double angle = getCurrentAngle();
    //     double velocitySetpoint = m_velocityControlEnabled ? m_velocitySetpoint : m_controller.getSetpoint().velocity;
    //     double positionSetpoint = m_velocityControlEnabled ? angle + velocitySetpoint * 0.02 : m_controller.getSetpoint().position;
    //     double accelerationSetpoint = m_velocityControlEnabled ? 0.0 : (velocitySetpoint - m_lastVelocitySetpoint) / (Timer.getFPGATimestamp() - m_lastTime);

    //     double feedforward = m_feedforward.calculate(positionSetpoint, velocitySetpoint, accelerationSetpoint);
    //     double positionPID = m_controller.calculate(angle);
    //     double velocityPID = m_velocityController.calculate(getCurrentVelocity(), velocitySetpoint);
    //     double pid = m_velocityControlEnabled ? velocityPID : positionPID;

    //     double currentEncoder = talon.getSelectedSensorPosition();
    //     System.out.println("FF: " + feedforward + " PID: " + pid);
    //     double voltage = RebelUtil.constrain(feedforward + pid, -12.0, 12.0);
    //     if (currentEncoder >= kUpperLimit && voltage > 0.0) {
    //         voltage = 0.0;
    //     } else if (currentEncoder <= kLowerLimit && voltage < 0.0) {
    //         voltage = 0.0;
    //     } 
        
    //     talon.setVoltage(voltage);

    //     m_lastVelocitySetpoint = velocitySetpoint;
    //     m_lastTime = Timer.getFPGATimestamp();

    // }
    
}
