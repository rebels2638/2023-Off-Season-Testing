package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private CANSparkMax m_motor = new CANSparkMax(1, MotorType.kBrushless); 

    private static final double kMotorToOutputShaftRatio = 0.25;
    private static final double kPulsePerRotation = 42;

    private static final double kSpikeAMPS = 10;


    private final PIDController feedBackController = new PIDController(0, 0, 0);
    private ArmFeedforward feedForwardController = new ArmFeedforward(0, 0, 0);

    private double goalVelocityRadSec = 0;

    private static final int currentLimit = 20;
    private static final double voltageLimit = 12;

    public Intake() {
        m_motor.setSmartCurrentLimit(currentLimit);
    }

    @Override
    public void periodic() {
        double feedForwardVoltage = feedForwardController.calculate(getVelocityRadSec(), goalVelocityRadSec);
        
        feedBackController.setSetpoint(goalVelocityRadSec);
        double feedBackControllerVoltage = feedBackController.calculate(getVelocityRadSec());
        double output = feedForwardVoltage + feedBackControllerVoltage;
        m_motor.setVoltage(Math.min(voltageLimit, output));
    }

    public double getVelocityRadSec() {
        return m_motor.getEncoder().getVelocity() * 2 * Math.PI/kPulsePerRotation * kMotorToOutputShaftRatio;
    }

    public void setVelocityRadSec(double angle) {
        goalVelocityRadSec = angle;
    }

    public boolean isSpike() {
        return m_motor.getOutputCurrent() >= kSpikeAMPS;
    }

}
