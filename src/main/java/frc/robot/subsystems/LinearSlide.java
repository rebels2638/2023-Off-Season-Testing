package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RebelUtil;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class LinearSlide extends SubsystemBase {
    private final WPI_TalonFX m_linslide;
    private boolean inverted = false;
    private static LinearSlide instance = null;
    private static final double kMaxEncoderLimit = 10000000;
    private static final double kMinEncoderLimit = -10000000;
    private double kS = 0.07;

    private ShuffleboardTab tab;
    private final GenericEntry linSlideEncoderPosition;

    private final PIDController pid = new PIDController(1.8 * (0.65 / 58000.0), 0, (0.65 / 58000.0) * (0.05));

    private boolean inPID = false;
    public boolean nearGrid = false;
    public double m_velocitySetpoint = 0.0;

    public LinearSlide() {
        this.m_linslide = new WPI_TalonFX(6);
        m_linslide.setNeutralMode(NeutralMode.Brake); // changed from brake

        tab = Shuffleboard.getTab("Linear Slide");
        inverted = false;
        linSlideEncoderPosition = tab.add("Lin_Encoder_Position", 0.0).getEntry();
        tab.add("Zero Encoder", new InstantCommand(() -> this.zeroEncoder()));
        tab.add("Max Out Encoder", new InstantCommand(() -> this.maxOutEncoder()));
        setGoal(0);
        inPID = false;
        pid.setTolerance(1000);
    }

    // Singleton class, call getInstance to access instead of the constructor.
    public static LinearSlide getInstance() {
        if (instance == null) {
            instance = new LinearSlide();
        }
        return instance;
    }

    public double getCurrentEncoderPosition() {
        return m_linslide.getSensorCollection().getIntegratedSensorPosition();
    }

    public double getCurrentEncoderRate() {
        return m_linslide.getSensorCollection().getIntegratedSensorVelocity() * 10; // motor velocity is in ticks per
                                                                                    // 100ms
    }

    public void setPID(boolean on) {
        pid.reset();
        inPID = on;
    }

    @Override
    public void periodic() {
        linSlideEncoderPosition.setDouble(m_linslide.getSensorCollection().getIntegratedSensorPosition());

        double percentOutput = 0.0;
        if (!(nearGrid && ElevatorPIDNonProfiled.getInstance().getCurrentHeight() < 0.6)) {

            double pidVoltage = inPID ? (pid.atSetpoint() ? 0.0 : pid.calculate(getCurrentEncoderPosition()))
                    : m_velocitySetpoint;
            double feedforward = (pidVoltage == 0 ? 0 : pidVoltage < 0 ? -1 : 1) * kS;
            percentOutput = feedforward + pidVoltage;

            if (getCurrentEncoderPosition() >= kMaxEncoderLimit && percentOutput > 0.0) {
                percentOutput = 0.0;
            } else if (getCurrentEncoderPosition() <= kMinEncoderLimit && percentOutput < 0.0) {
                percentOutput = 0.0;
            } else if (LinSlidePiston.getInstance().state) {
                percentOutput = 0.0;
            }
        }

        RebelUtil.constrain(percentOutput, -1.0, 1.0);
        m_linslide.set(ControlMode.PercentOutput, percentOutput * (inverted ? -1 : 1));
    }

    public void setVelocitySetpoint(double setpoint) {
        m_velocitySetpoint = setpoint;
    }

    public void setGoal(double encoderGoal) {
        pid.setSetpoint(encoderGoal);
    }

    public boolean atGoal() {
        return inPID ? pid.atSetpoint() : false;
    }

    public void zeroEncoder() {
        m_linslide.setSelectedSensorPosition(0);
    }

    public void maxOutEncoder() {
        m_linslide.setSelectedSensorPosition(56000);
    }
}
