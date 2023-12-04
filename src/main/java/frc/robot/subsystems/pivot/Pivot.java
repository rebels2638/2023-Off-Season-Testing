package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase{

    private CANSparkMax m_motor = new CANSparkMax(21, MotorType.kBrushless); 

    private static final double kMotorToOutputShaftRatio = 0.01;
    private static final double kPulsePerRotation = 42;

    private final PIDController feedBackController = new PIDController(0, 0, 0);
    private ArmFeedforward feedForwardController = new ArmFeedforward(0, 0, 0);

    private static final double kRadPositionTolerance = Math.toRadians(3);
    private double goalRadAngle = 0;

    public Pivot() {
        feedBackController.setTolerance(kRadPositionTolerance);
        m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    @Override
    public void periodic() {
        double feedForwardVoltage = feedForwardController.calculate(getRadAngle(), goalRadAngle);
        
        feedBackController.setSetpoint(goalRadAngle);
        double feedBackControllerVoltage = feedBackController.calculate(getRadAngle());
        SmartDashboard.putNumber("pivot/pivotMeasuredDegAngle", getDegAngle());

        m_motor.setVoltage(feedForwardVoltage + feedBackControllerVoltage);
    }

    public void setDegAngle(double angle) {
        goalRadAngle = angle;
    }

    public double getDegAngle() {
        return m_motor.getEncoder().getPosition() * (360 / kPulsePerRotation) * kMotorToOutputShaftRatio;
    }
    public double getRadAngle() {
        return m_motor.getEncoder().getPosition() * 2 * Math.PI/kPulsePerRotation * kMotorToOutputShaftRatio;
    }
    public boolean reachedSetpoint() {
        return feedBackController.atSetpoint();
    }
    public void zeroAngle() {
        m_motor.getEncoder().setPosition(0);
    }
}
