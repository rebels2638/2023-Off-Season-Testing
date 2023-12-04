package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase{

    private CANSparkMax m_motor = new CANSparkMax(21, MotorType.kBrushless); 
    private boolean velocityControlmode;

    private static final double kMotorToOutputShaftRatio = 0.01;
    private static final double kPulsePerRotation = 42;

    private final PIDController feedBackController = new PIDController(0, 0, 0); //TODO: modify for future 
    private ArmFeedforward feedForwardController = new ArmFeedforward(0, 0, 0); //TODO:  modify for future

    private final PIDController velocityPIDController = new PIDController(1,0,0); //TODO: modify for future
    private static final double kRadPositionTolerance = Math.toRadians(12);
    private double goalRadAngle = 0; // initials
    private double velocitySetPoint = 0;

    public Pivot() {
        feedBackController.setTolerance(kRadPositionTolerance);
        m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    @Override
    public void periodic() {
        if(!velocityControlmode){
        double feedForwardVoltage = feedForwardController.calculate(getRadAngle(), goalRadAngle);
        
        feedBackController.setSetpoint(goalRadAngle);
        double feedBackControllerVoltage = feedBackController.calculate(getRadAngle());
        updateShuffleBoard();
        SmartDashboard.putNumber("pivot/ExpectedOutputVoltage", feedForwardVoltage);
        driveVoltage(feedForwardVoltage + feedBackControllerVoltage);
        }
        else{
            if(velocityControlmode){
                
            }
        }
    }

    void updateShuffleBoard(){
        SmartDashboard.putNumber("pivot/pivotMeasuredDegAngle", getDegAngle());
        SmartDashboard.putNumber("pivot/pivotMeasuredRadAngle", getRadAngle());
    }

    public void setDegAngle(double angle) {
        if(goalRadAngle < 0){
            goalRadAngle = 0;
            return;
        }
        goalRadAngle = angle;
        return;
    }
    public void driveVoltage(double output){
        m_motor.setVoltage(output);
    }

    public void setVelocityControlMode(boolean b){  
        velocityControlmode = b;
    };

    public void setVelocitySetPoint(double setPoint){
        velocityPIDController.setSetpoint(setPoint);
        return;
    }


    public double getDegAngle() {
        return m_motor.getEncoder().getPosition() * 360 * kMotorToOutputShaftRatio;
    }
    public double getRadAngle() {
        return m_motor.getEncoder().getPosition() * 2 * Math.PI * kMotorToOutputShaftRatio;
    }
    public boolean reachedSetpoint() {
        return feedBackController.atSetpoint();
    }
    public void zeroAngle() {
        
        m_motor.getEncoder().setPosition(0);
    }
}
