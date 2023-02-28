package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {

    // private final WPI_TalonFX talon;
    private final WPI_TalonFX leftTalon;
    private final WPI_TalonFX rightTalon;
    
    private final MotorControllerGroup m_motorGroup;

    private static Elevator instance = null;
    private static double lastPercentSpeed; 
    private final DigitalInput limswitch;

    public Elevator() {
        // this.talon = new WPI_TalonFX(6); // one instance of TalonSRX, replaced IntakeConstants.TALON_ID
        this.leftTalon = new WPI_TalonFX(0);
        this.rightTalon = new WPI_TalonFX(3);
        this.m_motorGroup = new MotorControllerGroup(leftTalon, rightTalon);
        this.m_motorGroup.setInverted(true);

        lastPercentSpeed = 0;
        TalonFXConfiguration falconConfig = new TalonFXConfiguration();

        limswitch = new DigitalInput(0); // change this

        falconConfig.slot0.kP = 0;
        falconConfig.slot0.kI = 0;
        falconConfig.slot0.kD = 0;

        falconConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        falconConfig.voltageCompSaturation = 12;

        falconConfig.nominalOutputForward = 0;
        falconConfig.nominalOutputReverse = 0;
        falconConfig.peakOutputForward = 1;
        falconConfig.peakOutputReverse = -1;

        // talon.configAllSettings(falconConfig);
        // talon.setNeutralMode(NeutralMode.Coast);

        
        leftTalon.configAllSettings(falconConfig);
        leftTalon.setNeutralMode(NeutralMode.Coast);

        rightTalon.configAllSettings(falconConfig);
        rightTalon.setNeutralMode(NeutralMode.Coast);
        
    }

    // Singleton class, call getInstance to access instead of the constructor.
    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

    public void setVoltage(double voltage) {
        SmartDashboard.putNumber("ELEVATOR SUPPLY VOLTS", voltage);
        m_motorGroup.setVoltage(voltage);
    }

    public void resetEncoder() {    
        // talon.getSensorCollection().setIntegratedSensorPosition(0, 30);
    }

    public boolean getLimitSwitch() {return limswitch.get();}

    public double getEncoderValue() {return leftTalon.getSensorCollection().getIntegratedSensorPosition();}

    // public void setPosition(double thing) {talon.set(null, thing);, thing);;}

}