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
import frc.lib.RebelUtil;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class LinearSlide extends SubsystemBase {
    private final WPI_TalonFX m_linslide;
    private static LinearSlide instance = null;
    private static final double kMaxEncoderLimit = 10000000;
    private static final double kMinEncoderLimit = -10000000;

    private ShuffleboardTab tab;
    private double kG = 0;
    private final GenericEntry linSlideEncoderPosition;

    private double m_setpoint = 0.0;

    public LinearSlide() {
        this.m_linslide = new WPI_TalonFX(6);
        m_linslide.setNeutralMode(NeutralMode.Brake);

        tab = Shuffleboard.getTab("Linear Slide");
        linSlideEncoderPosition = tab.add("Lin_Encoder_Position", 0.0).getEntry();
        tab.add("Zero Encoder", new InstantCommand(() -> zeroEncoder()));
        m_setpoint = 0.0;
    }

    

    // Singleton class, call getInstance to access instead of the constructor.
    public static LinearSlide getInstance() {
        if (instance == null) {
            instance = new LinearSlide();
        }
        return instance;
    }

    public double getCurrentEncoderPosition() {
        return -m_linslide.getSensorCollection().getIntegratedSensorPosition();
    }

    public double getCurrentEncoderRate() {
        return -m_linslide.getSensorCollection().getIntegratedSensorVelocity() * 10; // motor velocity is in ticks per 100ms
    }

    @Override
    public void periodic() {
        linSlideEncoderPosition.setDouble(m_linslide.getSensorCollection().getIntegratedSensorPosition());
        m_linslide.set(ControlMode.PercentOutput, m_setpoint);

    }

    public void setPercentOutput(double percent) {
        m_setpoint = percent;
    }

    public void zeroEncoder() {
        m_linslide.setSelectedSensorPosition(0);
    }
}
