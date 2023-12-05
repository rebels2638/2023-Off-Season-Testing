package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

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
        goalRadAngle = angle;
    }

    public double getDegAngle() {
        return inputs.positionDeg;
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
