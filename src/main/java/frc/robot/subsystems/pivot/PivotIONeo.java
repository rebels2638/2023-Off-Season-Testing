package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotIONeo extends SubsystemBase implements PivotIO {
    private static final double kMotorToOutputShaftRatio = 0.01;
    private CANSparkMax m_motor = new CANSparkMax(21, MotorType.kBrushless); 

    private PIDController feedBackController = new PIDController(0, 0, 0);
    private ArmFeedforward feedForwardController = new ArmFeedforward(0, 0, 0);

    private double goalPositionRad = 0;

    public PivotIONeo() {
        m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.positionRad = m_motor.getEncoder().getPosition() * kMotorToOutputShaftRatio * Math.PI * 2;
        inputs.positionDeg = m_motor.getEncoder().getPosition() * kMotorToOutputShaftRatio * 360;
    }

    @Override
    // sould be called periodically
    public void setPosition(double goalPositionRad, double currentRadAngle) {
        double feedForwardVoltage = feedForwardController.calculate(goalPositionRad, 0);
        
        feedBackController.setSetpoint(goalPositionRad);
        double feedBackControllerVoltage = feedBackController.calculate(currentRadAngle);

        m_motor.setVoltage(feedForwardVoltage + feedBackControllerVoltage);
    } 

    @Override
    public void configureController(ArmFeedforward ff, PIDController fb) {
        feedBackController = fb;
        feedForwardController = ff;
    }

}