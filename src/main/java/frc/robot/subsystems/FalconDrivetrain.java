// this one is not used, it also does not work. FalconDrive.java calls DrivetrainFalcon.java now

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.motorcontrol.Encoder;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.utils.ConstantsFXDriveTrain.DriveConstants;
import frc.robot.utils.ConstantsFXDriveTrain.GearboxConstants;
import frc.lib.RebelUtil;
import frc.robot.Constants;
import frc.robot.Robot;

public class FalconDrivetrain extends SubsystemBase {
  public static final double kMaxSpeed = Constants.DrivetrainConstants.kMaxSpeed; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second in radians
  // public static final double kMaxAngularSpeedDegrees = 360;

  private static FalconDrivetrain instance = null;
  private static final double kTrackWidth = DriveConstants.TRACK_WIDTH_METERS; // meters
  private static final double kWheelRadiusStandard = GearboxConstants.WHEEL_DIAMETER_STANDARD / 2; // meters
  private static final double kWheelRadiusOmni = GearboxConstants.WHEEL_DIAMETER_OMNI / 2; // meters
  private static final int kEncoderResolution = 2048;
  private boolean inHighGear = true;

  private double kNativeUnitsPerRotation = kEncoderResolution * getGearingRatio();
  private double kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
  private double kMetersPerRotationStandard = 2 * Math.PI * kWheelRadiusStandard;
  private double kMetersPerRotationOmni = 2 * Math.PI * kWheelRadiusOmni;
  private double kMetersPerRotationFront = (kMetersPerRotationStandard + kMetersPerRotationOmni) / 2.0;
  private double kMetersPerRotationBack = (kMetersPerRotationStandard + kMetersPerRotationStandard) / 2.0;
  private double kRotationsPerMeter = 1 / kMetersPerRotationBack;
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

  private final Solenoid m_leftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 14);
  private final Solenoid m_rightSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 15);

  // private final Gyro m_gyro = new AHRS(Port.kUSB);

  private final PIDController m_leftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI,
      DriveConstants.kD);
  private final PIDController m_rightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI,
      DriveConstants.kD);

  private double m_leftSetpoint = 0.0;
  private double m_rightSetpoint = 0.0;

  // private final DifferentialDrive m_drive = new DifferentialDrive(m_leftGroup,
  // m_rightGroup);

  private final ShuffleboardTab tab;

  private final GenericEntry kGHigh;
  private final GenericEntry kGLow;

  private final GenericEntry leftMotorEncoderPosition;
  private final GenericEntry leftMotorPosition;
  private final GenericEntry leftMotorVelocity;
  private final GenericEntry leftMotorVelocitySetpoint;
  private final GenericEntry leftMotorVoltageSupplied;
  private final GenericEntry leftMotorVoltageSetpoint;

  private final GenericEntry rightMotorEncoderPosition;
  private final GenericEntry rightMotorPosition;
  private final GenericEntry rightMotorVelocity;
  private final GenericEntry rightMotorVelocitySetpoint;
  private final GenericEntry rightMotorVoltageSupplied;
  private final GenericEntry rightMotorVoltageSetpoint;

  private final GenericEntry gyroPitch;
  private final GenericEntry gyroAngle;
  private final GenericEntry poseString;

  private double m_leftVoltageSetpoint;
  private double m_rightVoltageSetpoint;

  private double leftMetersTraveled = 0.0;
  private double rightMetersTraveled = 0.0;

  private double prevLeftBack = 0;
  private double prevLeftFront = 0;
  private double prevRightFront = 0;
  private double prevRightBack = 0;

  private boolean isBalancing = false;

  public final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

  // private final DifferentialDriveOdometry m_odometry;

  private SimpleMotorFeedforward m_feedforwardHigh = new SimpleMotorFeedforward(GearboxConstants.STATIC_GAIN_HIGH,
      GearboxConstants.VELOCITY_GAIN_HIGH, GearboxConstants.ACCEL_GAIN_HIGH);
  private SimpleMotorFeedforward m_feedforwardLow = new SimpleMotorFeedforward(GearboxConstants.STATIC_GAIN_LOW,
      GearboxConstants.VELOCITY_GAIN_LOW, GearboxConstants.ACCEL_GAIN_LOW);
  public SimpleMotorFeedforward m_feedforward = inHighGear ? m_feedforwardHigh : m_feedforwardLow;
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
  private Gyro m_gyroSim;
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;

  public FalconDrivetrain() {
    switchToHighGear();
    isBalancing = true;

    leftMetersTraveled = 0.0;
    rightMetersTraveled = 0.0;
    prevLeftBack = 0;
    prevLeftFront = 0;
    prevRightFront = 0;
    prevRightBack = 0;

    m_leftLeader.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.setNeutralMode(NeutralMode.Brake);
    m_rightLeader.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.setNeutralMode(NeutralMode.Brake);

    m_leftGroup.setInverted(DriveConstants.FALCON_LEFT_GROUP_INVERTED);
    m_rightGroup.setInverted(DriveConstants.FALCON_RIGHT_GROUP_INVERTED);
    zeroEncoder();
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    tab = Shuffleboard.getTab("Drive");
    kGLow = tab.add("kG Low", 0.0).getEntry();
    kGHigh = tab.add("kG High", 0.0).getEntry();
    leftMotorEncoderPosition = tab.add("Left Encoder Position", 0.0).getEntry();
    leftMotorPosition = tab.add("Left Meters", 0.0).getEntry();
    leftMotorVelocity = tab.add("Left Velocity", 0.0).getEntry();
    leftMotorVelocitySetpoint = tab.add("Left Velocity Setpoint", 0.0).getEntry();
    leftMotorVoltageSupplied = tab.add("Left Motor Voltage", 0.0).getEntry();
    leftMotorVoltageSetpoint = tab.add("Left Voltage Setpoint", 0.0).getEntry();

    rightMotorEncoderPosition = tab.add("Right Encoder Position", 0.0).getEntry();
    rightMotorPosition = tab.add("Right Meters", 0.0).getEntry();
    rightMotorVelocity = tab.add("Right Velocity", 0.0).getEntry();
    rightMotorVelocitySetpoint = tab.add("Right Velocity Setpoint", 0.0).getEntry();
    rightMotorVoltageSupplied = tab.add("Right Motor Voltage", 0.0).getEntry();
    rightMotorVoltageSetpoint = tab.add("Right Voltage Setpoint", 0.0).getEntry();

    gyroPitch = tab.add("Gyro Pitch", 0.0).getEntry();
    gyroAngle = tab.add("Gyro Angle", 0.0).getEntry();

    poseString = tab.add("Pose String", "").getEntry();

    if (RobotBase.isSimulation()) {
      // TODO: EDIT VALUES TO BE ACCURATE
      m_differentialDrivetrainSimulator = new DifferentialDrivetrainSim(m_drivetrainSystem,
          DCMotor.getCIM(4),
          26.667, kTrackWidth,
          kWheelRadiusStandard,
          null);
      // m_gyroSim = m_gyro;
      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    }

    tab.add("Zero Encoders", new InstantCommand(() -> zeroEncoder()));
    tab.add("Zero Gyro", new InstantCommand(() -> zeroHeading()));
  }

  public static FalconDrivetrain getInstance() {
    if (instance == null) {
      instance = new FalconDrivetrain();
    }
    return instance;
  }

  public double metersToNative(double meters) {
    return meters * kRotationsPerMeter * kNativeUnitsPerRotation;
  }

  public double nativeToMeters(double encoderUnits) {
    return encoderUnits * kRotationsPerNativeUnit * (getPitch() > 0 ? kMetersPerRotationBack : kMetersPerRotationFront);
  }

  public double getGearingRatio() {
    return inHighGear ? GearboxConstants.HIGH_GEAR_REDUCTION : GearboxConstants.LOW_GEAR_REDUCTION;
  }

  public void recalcConversionFactors() {
    kNativeUnitsPerRotation = kEncoderResolution * getGearingRatio();
    kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
  }

  public double getCurrentEncoderPosition(WPI_TalonFX motor) {
    return motor.getSensorCollection().getIntegratedSensorPosition();
  }

  public double getCurrentEncoderRate(WPI_TalonFX motor) {
    return motor.getSensorCollection().getIntegratedSensorVelocity() * 10; // motor velocity is in ticks per 100ms
  }

  public double getLeftSideMeters() {
    return leftMetersTraveled;
  }

  public double getRightSideMeters() {
    return rightMetersTraveled;
  }

  public double getLeftSideVelocity() {
    return (nativeToMeters(-getCurrentEncoderRate(m_leftLeader))
        + nativeToMeters(-getCurrentEncoderRate(m_leftFollower))) / 2.0;
  }

  public double getRightSideVelocity() {
    return (nativeToMeters(getCurrentEncoderRate(m_rightLeader))
        + nativeToMeters(getCurrentEncoderRate(m_rightFollower))) / 2.0;
  }

  public double getLeftSideDiff() {
    double leftFront = -getCurrentEncoderPosition(m_leftLeader);
    double leftBack = -getCurrentEncoderPosition(m_leftFollower);

    double traveledDiff = (nativeToMeters(leftFront - prevLeftFront)
        + nativeToMeters(leftBack - prevLeftBack)) / 2.0;

    prevLeftFront = leftFront;
    prevLeftBack = leftBack;

    return traveledDiff;
  }

  public double getRightSideDiff() {
    double rightFront = getCurrentEncoderPosition(m_rightLeader);
    double rightBack = getCurrentEncoderPosition(m_rightFollower);

    double traveledDiff = (nativeToMeters(rightFront - prevRightFront)
        + nativeToMeters(rightBack - prevRightBack)) / 2.0;

    prevRightFront = rightFront;
    prevRightBack = rightBack;
    
    return traveledDiff;
  }

  public void setBalancing(boolean balancing) {
    isBalancing = balancing;
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    m_leftSetpoint = speeds.leftMetersPerSecond;
    m_rightSetpoint = speeds.rightMetersPerSecond;
    var kG = inHighGear ? kGHigh.getDouble(0.0) : kGLow.getDouble(0.0);
    var kGReal = 4.44;
    var kGDiff = -1.09;
    var kGFinal = kGReal + (PoseEstimator.getInstance().getPitch() > 0.0 ? 1.0 : -1.0) * kGDiff;
    if (!isBalancing)
      kGFinal *= -1;
    if (Math.abs(PoseEstimator.getInstance().getPitch()) < 5.0)
      kGFinal = 0.0;
    var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond)
        + kGFinal * Math.sin(PoseEstimator.getInstance().getPitch() * (Math.PI / 180.0));
    var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond)
        + kGFinal * Math.sin(PoseEstimator.getInstance().getPitch() * (Math.PI / 180.0));
    double leftPID = m_leftPIDController
        .calculate(getLeftSideVelocity(), speeds.leftMetersPerSecond);
    double rightPID = m_rightPIDController
        .calculate(getRightSideVelocity(), speeds.rightMetersPerSecond);
    m_leftVoltageSetpoint = leftFeedforward + leftPID;
    m_rightVoltageSetpoint = rightFeedforward + rightPID;

    RebelUtil.constrain(m_leftVoltageSetpoint, -12, 12);
    RebelUtil.constrain(m_rightVoltageSetpoint, -12, 12);
    m_leftGroup.setVoltage(m_leftVoltageSetpoint);
    m_rightGroup.setVoltage(m_rightVoltageSetpoint);
  }

  public void updateSmartDashBoard() {
    SmartDashboard.putNumber("AverageEncoderDistance", this.getAverageEncoderDistance());
    if (Robot.isSimulation())
      SmartDashboard.putNumber("CurrentDrawnAmps", m_differentialDrivetrainSimulator.getCurrentDrawAmps());

    m_fieldSim.setRobotPose(getPose());
    SmartDashboard.putData("Field", m_fieldSim);
    SmartDashboard.putNumber("leftGroup Diff", nativeToMeters(getCurrentEncoderRate(m_leftLeader)));
    SmartDashboard.putNumber("rightGroup Diff", nativeToMeters(-getCurrentEncoderRate(m_rightLeader)));
  }

  public void updateShuffleboard() {
    leftMotorEncoderPosition.setDouble(-getCurrentEncoderPosition(m_leftLeader));
    leftMotorPosition.setDouble(getLeftSideMeters());
    leftMotorVelocity.setDouble(getLeftSideVelocity());
    leftMotorVelocitySetpoint.setDouble(m_leftSetpoint);
    leftMotorVoltageSetpoint.setDouble(m_leftLeader.getMotorOutputVoltage());
    leftMotorVoltageSupplied.setDouble(m_leftVoltageSetpoint);

    rightMotorEncoderPosition.setDouble(getCurrentEncoderPosition(m_rightLeader));
    rightMotorPosition.setDouble(getRightSideMeters());
    rightMotorVelocity.setDouble(getRightSideVelocity());
    rightMotorVelocitySetpoint.setDouble(m_rightSetpoint);
    rightMotorVoltageSetpoint.setDouble(m_rightLeader.getMotorOutputVoltage());
    rightMotorVoltageSupplied.setDouble(m_rightVoltageSetpoint);

    gyroAngle.setDouble(getHeading());
    gyroPitch.setDouble(PoseEstimator.getInstance().getPitch());

    poseString.setString(PoseEstimator.getInstance().getFormattedPose());
  }

  public void drive(double xSpeed, double rot) {
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot)));
  }

  public void resetOdometry(Pose2d pose) {
    zeroEncoder();
    if (Robot.isSimulation())
      m_differentialDrivetrainSimulator.setPose(pose);
  }

  public void zeroEncoder() {
    m_leftLeader.getSensorCollection().setIntegratedSensorPosition(0, 30);
    m_leftFollower.getSensorCollection().setIntegratedSensorPosition(0, 30);
    m_rightLeader.getSensorCollection().setIntegratedSensorPosition(0, 30);
    m_rightFollower.getSensorCollection().setIntegratedSensorPosition(0, 30);
  }

  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  public double getDrawnCurrentAmps() {
    return m_differentialDrivetrainSimulator.getCurrentDrawAmps();
  }

  public Pose2d getPose() {
    return PoseEstimator.getInstance().getCurrentPose();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  @Override
  public void periodic() {
    this.leftMetersTraveled += getLeftSideDiff();
    this.rightMetersTraveled += getRightSideDiff();

    this.updateSmartDashBoard();
    this.updateShuffleboard();
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
    // m_gyroSim.setAngle(-m_differentialDrivetrainSimulator.getHeading().getDegrees());
  }

  public void zeroHeading() {
    // m_gyro.reset();
    PoseEstimator.getInstance().resetHeading();
  }

  public double getHeading() {
    // return Math.IEEEremainder(m_gyro.getAngle(), 360) * 1; // Multiply by -1 if
    return PoseEstimator.getInstance().getAngle();
    // the GYRO is REVERSED.
  }

  public double getPitch() {
    // return Math.IEEEremainder(m_gyro.getAngle(), 360) * 1; // Multiply by -1 if
    return PoseEstimator.getInstance().getPitch();
    // the GYRO is REVERSED.
  }

  public Rotation2d getRotation2d() {
    // return m_gyro.getRotation2d();
    return PoseEstimator.getInstance().getCurrentPose().getRotation();
  }

  /*
   * public void tankDriveVolts(double leftVolts, double rightVolts) {
   * m_leftLeader.setVoltage(leftVolts);
   * m_rightLeader.setVoltage(rightVolts);
   * m_drive.feed();
   * }
   */

  public void setVoltageFromAuto(double leftVoltage, double rightVoltage) {
    var kGReal = 4.44;
    var kGDiff = -1.09;
    var kGFinal = kGReal + (PoseEstimator.getInstance().getPitch() > 0.0 ? 1.0 : -1.0) * kGDiff;
    if (!isBalancing)
      kGFinal *= -1;
    if (Math.abs(PoseEstimator.getInstance().getPitch()) < 5.0)
      kGFinal = 0.0;
    m_leftGroup
        .setVoltage(leftVoltage + kGFinal * Math.sin(PoseEstimator.getInstance().getPitch() * (Math.PI / 180.0)));
    m_rightGroup
        .setVoltage(rightVoltage + kGFinal * Math.sin(PoseEstimator.getInstance().getPitch() * (Math.PI / 180.0)));
  }

  public void switchToHighGear() {
    m_leftSolenoid.set(true);
    m_rightSolenoid.set(true);
    inHighGear = true;
    recalcConversionFactors();
    m_feedforward = m_feedforwardHigh;
  }

  public void switchToLowGear() {
    m_leftSolenoid.set(false);
    m_rightSolenoid.set(false);
    inHighGear = false;
    recalcConversionFactors();
    m_feedforward = m_feedforwardLow;
  }
}
