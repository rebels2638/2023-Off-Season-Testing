package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.kauailabs.navx.frc.AHRS; 

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.motorcontrol.Encoder;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.utils.ConstantsFXDriveTrain.DriveConstants;
import frc.robot.utils.ConstantsFXDriveTrain.GearboxConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class FalconDrivetrain extends SubsystemBase {
  public static final double kMaxSpeed = Constants.DrivetrainConstants.kMaxSpeed; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second in radians
  // public static final double kMaxAngularSpeedDegrees = 360;

  private static final double kTrackWidth = DriveConstants.TRACK_WIDTH_METERS; // meters
  private static final double kWheelRadius = GearboxConstants.WHEEL_DIAMETER / 2; // meters
  private static final int kEncoderResolution = 2048;
  /*
   * =============================================================================
   * ==============
   */
  private final WPI_TalonFX m_leftLeader = new WPI_TalonFX(DriveConstants.FALCON_LEFT_FRONT_ID);
  private final WPI_TalonFX m_leftFollower = new WPI_TalonFX(DriveConstants.FALCON_LEFT_BACK_ID);
  private final WPI_TalonFX m_rightLeader = new WPI_TalonFX(DriveConstants.FALCON_RIGHT_FRONT_ID);
  private final WPI_TalonFX m_rightFollower = new WPI_TalonFX(DriveConstants.FALCON_RIGHT_BACK_ID);

  private final Encoder m_leftEncoder = new Encoder(5, 6);
  private final Encoder m_rightEncoder = new Encoder(7, 8);

  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftLeader, m_leftFollower);
  private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightLeader, m_rightFollower);

  private final AnalogGyro m_gyro = new AnalogGyro(Constants.GyroConstants.kGyroPort);

  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
  /*
   * =============================================================================
   * ================
   */
  private final Field2d m_fieldSim = new Field2d();
  private static final double KVlinear = 0.5;
  private static final double KAlinear = 0.5;
  private static final double KVAngular = 0.5;
  private static final double KAAngular = 0.5;
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(KVlinear,
      KAlinear, KVAngular, KAAngular);
  private DifferentialDrivetrainSim m_differentialDrivetrainSimulator;
  private AnalogGyroSim m_gyroSim;
  // private final static AHRS gyro = new AHRS(SPI.Port.kMXP);
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;

  public FalconDrivetrain() {
    m_leftGroup.setInverted(DriveConstants.FALCON_LEFT_GROUP_INVERTED);
    m_rightGroup.setInverted(DriveConstants.FALCON_RIGHT_GROUP_INVERTED);
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_leftLeader.getSensorCollection().setIntegratedSensorPosition(0, 30);
    m_rightLeader.getSensorCollection().setIntegratedSensorPosition(0, 30);
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    
    if (RobotBase.isSimulation()) {
      // TODO: EDIT VALUES TO BE ACCURATE
      m_differentialDrivetrainSimulator = new DifferentialDrivetrainSim(m_drivetrainSystem,
          DCMotor.getCIM(4),
          36, kTrackWidth,
          kWheelRadius,
          null);
      m_gyroSim = new AnalogGyroSim(m_gyro);
      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);
      // m_gyroSim = new AHRS(SPI.Port.KXMP);
    }
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
        final double leftOutput =
        m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

    
    //double leftOutput = m_leftPIDController.calculate(m_leftLeader.getSensorCollection().getIntegratedSensorPosition(), speeds.leftMetersPerSecond);
    //double rightOutput = m_rightPIDController.calculate(m_rightLeader.getSensorCollection().getIntegratedSensorPosition(), speeds.rightMetersPerSecond);

    System.out.println("VOLTAGE " + (leftFeedforward + leftOutput) + " RIGHT VOLTAGE " + (rightOutput + rightFeedforward));
    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  public void updateSmartDashBoard() {
    SmartDashboard.putNumber("AverageEncoderDistance", this.getAverageEncoderDistance());
    if(Robot.isSimulation()) SmartDashboard.putNumber("CurrentDrawnAmps", m_differentialDrivetrainSimulator.getCurrentDrawAmps());

    m_fieldSim.setRobotPose(getPose());
    SmartDashboard.putData("Field", m_fieldSim);
    SmartDashboard.putNumber("leftGroup Speed", m_leftGroup.get());
    SmartDashboard.putNumber("rightGroup speed", m_rightGroup.get());

  }

  public void drive(double xSpeed, double rot) {
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot)));
  }

  public void updateOdometry() {
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_differentialDrivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(pose.getRotation(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public double getDrawnCurrentAmps() {
    return m_differentialDrivetrainSimulator.getCurrentDrawAmps();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  @Override
  public void periodic() {
    this.updateOdometry();
    this.updateSmartDashBoard();
  }

  @Override
  public void simulationPeriodic() {
    m_differentialDrivetrainSimulator.setInputs(m_leftGroup.get() * RobotController.getInputVoltage(),
        m_rightGroup.get() * RobotController.getInputVoltage());
    m_differentialDrivetrainSimulator.update(0.02);

    m_leftEncoderSim.setDistance(m_differentialDrivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_differentialDrivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_differentialDrivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_differentialDrivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_differentialDrivetrainSimulator.getHeading().getDegrees());
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * 1; // Multiply by -1 if the GYRO is REVERSED.
  }

}
