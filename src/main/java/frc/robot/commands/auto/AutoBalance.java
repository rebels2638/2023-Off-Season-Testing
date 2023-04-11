// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.subsystems.FalconDrivetrain;
import frc.robot.subsystems.PoseEstimator;

import com.ctre.phoenix.CANifier.PWMChannel;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class AutoBalance extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final FalconDrivetrain m_driveTrain;
	// private final SerialPort sPort = new SerialPort(200, Port.kUSB);

	public static final double RADS = Math.PI/180;

	private final PoseEstimator poseEstimatorSubsystem;
	private final double yawErrorMargin = 5 * RADS;
	private final double yawVeloErrorMargin = 5 * RADS;
	private final double pitchErrorMargin = 2 * RADS;
	private final double pitchVeloErrorMargin = 5 * RADS;
	private final double pitchAccelErrorMargin = 0.5 * RADS;

	private final double rkp = 7; // r = rotation
	private final double rki = 0;
	private final double rkd = 0;

	private double dkp = -1.15; // originally -1.25
	private double dki = 0; // d = degrees relative to ground
	private double dkd = 0;

	private double dk2p = 0;
	private double dk2i = 0;
	private double dk2d = -1;

	private double m_headingSetpoint;
	// private boolean bBalanced = false;

	private PIDController rpidController;
	private PIDController dpidController;
	private PIDController dpid2Controller;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public AutoBalance(FalconDrivetrain drive, PoseEstimator pose) {
		m_driveTrain = drive;
		poseEstimatorSubsystem = pose;
		rpidController = new PIDController(rkp, rki, rkd);
		dpidController = new PIDController(dkp, dki, dkd);
		dpid2Controller = new PIDController(dk2p, dk2i, dk2d);
		rpidController.enableContinuousInput(-Math.PI, Math.PI);
		dpidController.enableContinuousInput(-Math.PI, Math.PI);
		dpid2Controller.enableContinuousInput(-Math.PI, Math.PI);

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drive, pose);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_driveTrain.setBalancing(true);
		SmartDashboard.putNumber("Balance kp", 0);
		SmartDashboard.putNumber("Balance kd", 0);
		rpidController.setTolerance(yawErrorMargin, yawVeloErrorMargin);
		dpidController.setTolerance(pitchErrorMargin, pitchVeloErrorMargin);
		dpid2Controller.setTolerance(pitchVeloErrorMargin, pitchAccelErrorMargin);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// dpidController.setP(dkp);
		// dpidController.setD(dkd);
		Pose2d currentPose = poseEstimatorSubsystem.getCurrentPose();
		double currentRot = currentPose.getRotation().getRadians();
		m_headingSetpoint = 0.0;

		if (Math.cos(currentRot) < 0.0)
			{m_headingSetpoint = Math.PI;}
			

		rpidController.setSetpoint(m_headingSetpoint);
		dpidController.setSetpoint(0.0);
		dpid2Controller.setSetpoint(0.0);

		double rpidVoltage = rpidController.calculate(currentRot);
		double dpidVoltage = dpidController.calculate(poseEstimatorSubsystem.getPitch());
		
		double veloErr = dpidController.getVelocityError();
		
		double dpid2Voltage = dpid2Controller.calculate(veloErr);

		if (!rpidController.atSetpoint()) {
			m_driveTrain.drive(0, rpidVoltage);
		} else if (!dpidController.atSetpoint()) {
			m_driveTrain.drive(dpidVoltage /*+ dpid2Voltage*/, 0);
		} else {
			m_driveTrain.drive(0, 0);
		}

		// System.out.println("In autobalance command");
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
