package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class LinearSlide extends SubsystemBase {
    private final WPI_TalonFX m_linslide;
    
    private static LinearSlide instance = null;

    // private final ShuffleboardTab tab;

    // private final GenericEntry linSlideEncoderPosition;
    // private final GenericEntry linSlidePosition;
    // private final GenericEntry linSlideVelocity;
    // private final GenericEntry linSlideAcceleration;
    // private final GenericEntry linSlidePositionSetpoint;
    // private final GenericEntry linSlideVelocitySetpoint;
    // private final GenericEntry linSlideAccelerationSetpoint;
    // private final GenericEntry voltageSupplied;
    // private final GenericEntry voltageSetpoint;

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

    public LinearSlide() {
        this.m_linslide = new WPI_TalonFX(7); // change when found
        m_linslide.setNeutralMode(NeutralMode.Coast);

        // tab = Shuffleboard.getTab("Elevator");
        // linSlideEncoderPosition = tab.add("Encoder Position", 0.0).getEntry();
        // linSlidePosition = tab.add("Height", 0.0).getEntry();
        // linSlideVelocity = tab.add("Velocity", 0.0).getEntry();
        // linSlideAcceleration = tab.add("Acceleration", 0.0).getEntry();
        // linSlidePositionSetpoint = tab.add("Height Setpoint", 0.0).getEntry();
        // linSlideVelocitySetpoint = tab.add("Velocity Setpoint", 0.0).getEntry();
        // linSlideAccelerationSetpoint = tab.add("Acceleration Setpoint", 0.0).getEntry();
        // voltageSupplied = tab.add("Motor Voltage", 0.0).getEntry();
        // voltageSetpoint = tab.add("Voltage Setpoint", 0.0).getEntry();

        // tab.add("Zero Encoder",
        //         new InstantCommand(() -> zeroEncoder()));
    }

    // Singleton class, call getInstance to access instead of the constructor.
    public static LinearSlide getInstance() {
        if (instance == null) {
            instance = new LinearSlide();
        }
        return instance;
    }

    public void zeroEncoder() {
        m_linslide.getSensorCollection().setIntegratedSensorPosition(0, 30);
    }

    public double heightToNative(double heightUnits) {
        return heightUnits * kRotationsPerMeter * kNativeUnitsPerRotation;
    }

    public double nativeToHeight(double encoderUnits) {
        return encoderUnits * kRotationsPerNativeUnit * kMetersPerRotation;
    }


    public double getCurrentEncoderPosition() {
        return m_linslide.getSensorCollection().getIntegratedSensorPosition();
    }

    public double getCurrentEncoderRate() {
        return m_linslide.getSensorCollection().getIntegratedSensorVelocity() * 10; // motor velocity is in ticks per 100ms
    }

    public double getCurrentHeight() {
        return nativeToHeight(getCurrentEncoderPosition());
    }

    public double getCurrentVelocity() {
        return nativeToHeight(getCurrentEncoderRate());
    }

    // public void updateShuffleboard() {
    //     linSlideEncoderPosition.setDouble(getCurrentEncoderPosition());
    //     linSlidePosition.setDouble(getCurrentHeight());
    //     linSlideVelocity.setDouble(getCurrentVelocity());
    //     voltageSupplied.setDouble(m_linslide.getMotorOutputVoltage());
    // }

    public void setPercentOutput(double percent) {
        m_linslide.set(ControlMode.PercentOutput, percent); // set talon speed based on input from XboxController.getleftY(), ie the input range on left y should map to the speed???? where speed is in range -1,1 and the xbox controller left joy stick is also -1,1???
    }

    @Override
    public void periodic() {
        // updateShuffleboard();
    }
}