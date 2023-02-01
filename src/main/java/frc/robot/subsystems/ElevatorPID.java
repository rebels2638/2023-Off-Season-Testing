package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/** Elevator subsystem with feed-forward and PID for position */
public class ElevatorPID extends SubsystemBase {
    public static final double kMaxSpeed =2.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

    private static final double kWheelRadius = 0.03; // meters
    private static final int kEncoderResolution = 2048;
    private static final int kGearingRatio = 100;
        
    public static final double kP = 1.8924; // for position loop: 252.69
    public static final double kI = 0; 
    public static final double kD = 0; // for position loop: 80.731

    public static final double kS = 0.027516;
    public static final double kV = 58.275;
    public static final double kA = 6.2261;

    private static final double kNativeUnitsPerRotation = kEncoderResolution * kGearingRatio;
    private static final double kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
    private static final double kMetersPerRotation = 2 * Math.PI * kWheelRadius;
    private static final double kRotationsPerMeter = 1 / kMetersPerRotation;

    private final WPI_TalonFX m_motor = new WPI_TalonFX(6);

    public final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAngularSpeed));
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(kS, kV);

    private TrapezoidProfile.State m_setpoint;
    private double feedforward;
    private double pid;
    private TrapezoidProfile.State goal;
    private double velocitySetpoint;
        
    // reset elevator (make sure to just like. push it to the bottom thx)
    public ElevatorPID() {
        m_motor.setInverted(true);
        m_motor.set(ControlMode.PercentOutput, 0);
        setSetpoint(0);
        m_motor.getSensorCollection().setIntegratedSensorPosition(0, 30); // reset encoders
    //  encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative); (i dont think this does anything????)
    }

    /*
    * Convert from TalonFX elevator position to native units
    */
    public double heightToNative(double heightUnits) {
        return heightUnits * kRotationsPerMeter * kNativeUnitsPerRotation;
    }

    public double nativeToHeight(double encoderUnits) {
        
        return encoderUnits * kRotationsPerNativeUnit * kMetersPerRotation;
    }

    // set setpoint
    public void setSetpoint(double setpoint) // setpoint in meters
    {
        //m_setpoint = new TrapezoidProfile.State(setpoint, 0);
        velocitySetpoint = setpoint;
    }

    /*
    * Compute voltages using feedforward and pid
    */
    @Override
    public void periodic() {           
        //m_controller.setGoal(m_setpoint); 
        //goal = m_controller.getSetpoint();
        feedforward = m_feedforward.calculate(velocitySetpoint); //goal.velocity

        double currentVelocity = nativeToHeight(m_motor.getSensorCollection().getIntegratedSensorVelocity());

        // System.out.println("voltage: " + (feedforward + pid) + "    encoder: " + m_motor.getSensorCollection().getIntegratedSensorVelocity());
        pid = m_controller.calculate(currentVelocity, velocitySetpoint);
        // System.out.println(feedforward + pid);

        m_motor.setVoltage(feedforward + pid);
    }
}
