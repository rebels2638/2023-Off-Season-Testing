package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotIONeo extends SubsystemBase implements PivotIO {
    private static final double kMotorToOutputShaftRatio = 0.01;
    private CANSparkMax m_motor = new CANSparkMax(21, MotorType.kBrushless); 

    private PIDController positionFeedBackController = new PIDController(0, 0, 0);
    private ArmFeedforward positionFeedForwardController = new ArmFeedforward(0, 0, 0);
    private PIDController velocityFeedBackController = new PIDController(0, 0, 0);
    private ArmFeedforward velocityFeedForwardController = new ArmFeedforward(0, 0, 0);

    public PivotIONeo() {
        m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.positionRad = m_motor.getEncoder().getPosition() * kMotorToOutputShaftRatio * Math.PI * 2;
        inputs.positionDeg = m_motor.getEncoder().getPosition() * kMotorToOutputShaftRatio * 360;

        inputs.velocityRadSec = m_motor.getEncoder().getVelocity() / 60 * kMotorToOutputShaftRatio * Math.PI * 2;
        inputs.velocityDegSec = m_motor.getEncoder().getVelocity() / 60 * kMotorToOutputShaftRatio * 360;
    }

    @Override
    // sould be called periodically
    public void setPosition(double goalPositionRad, double currentRadAngle) {
        double feedForwardVoltage = positionFeedForwardController.calculate(goalPositionRad, 0);
        
        positionFeedBackController.setSetpoint(goalPositionRad);
        double feedBackControllerVoltage = positionFeedBackController.calculate(currentRadAngle);

        m_motor.setVoltage(feedForwardVoltage + feedBackControllerVoltage);
    } 

    @Override
    // sould be called periodically
    public void setVelocity(double goalVelocityRadPerSec, double currentVelocityRadPerSec) {
        double feedForwardVoltage = velocityFeedForwardController.calculate(goalVelocityRadPerSec, 0);
        
        velocityFeedBackController.setSetpoint(goalVelocityRadPerSec);
        double feedBackControllerVoltage = velocityFeedBackController.calculate(currentVelocityRadPerSec);

        m_motor.setVoltage(feedForwardVoltage + feedBackControllerVoltage);
    } 

    @Override
    public void configureController(ArmFeedforward pff, PIDController pfb, 
                                        ArmFeedforward vff, PIDController vfb ) {
        positionFeedBackController = pfb;
        positionFeedForwardController = pff;
        velocityFeedBackController = vfb;
        velocityFeedForwardController = vff;
    }

    @Override
    public boolean reachedSetpoint(boolean isPositionalControll) {
        if (isPositionalControll) {
            return positionFeedBackController.atSetpoint();
        }
        return velocityFeedBackController.atSetpoint();
    }

}