package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase{

    private static final double kRadPositionTolerance = Math.toRadians(8);
    private double goalRadAngle = 0;

    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public Pivot(PivotIO io) {
        this.io = io;
        if (RobotBase.isReal()) {
            PIDController feedBackController = new PIDController(0, 0, 0);
            ArmFeedforward feedForwardController = new ArmFeedforward(0, 0, 0);
            feedBackController.setTolerance(kRadPositionTolerance);

            io.configureController(new ArmFeedforward(0, 0, 0), new PIDController(0, 0, 0));
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Pivot", inputs);

        io.setPosition(inputs.positionRad, goalRadAngle);
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
        return m_motor.getEncoder().getPosition() * (360 / kPulsePerRotation) * kMotorToOutputShaftRatio;
    }
    public double getRadAngle() {
        return inputs.positionRad;
    }
    public void zeroAngle() {
        inputs.positionDeg = 0;
        inputs.positionRad = 0;
    }
    public boolean reachedSetpoint() {
        return io.reachedSetpoint();
    }
}
