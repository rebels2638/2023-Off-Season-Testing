package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class LinearSlide extends SubsystemBase {
    private final WPI_TalonFX m_linslide;
    private static LinearSlide instance = null;
    private static final double kMaxEncoderLimit = 100;
    private static final double kMinEncoderLimit = 10;

    public LinearSlide() {
        this.m_linslide = new WPI_TalonFX(6); // change when found
        m_linslide.setNeutralMode(NeutralMode.Brake);
    }

    // Singleton class, call getInstance to access instead of the constructor.
    public static LinearSlide getInstance() {
        if (instance == null) {
            instance = new LinearSlide();
        }
        return instance;
    }

    public void setPercentOutput(double percent) {

        if ((percent > 0.0 && kMaxEncoderLimit >= m_linslide.getSelectedSensorPosition()) || (percent < 0.0 && kMinEncoderLimit <= m_linslide.getSelectedSensorPosition())) {
            percent = 0; 
        }
        m_linslide.set(ControlMode.PercentOutput, percent);

    }
}
