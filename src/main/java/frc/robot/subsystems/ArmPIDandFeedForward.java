package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmPIDandFeedForward extends SubsystemBase {
    private final WPI_TalonFX talon;
    private static Arm instance = null;
  
  
    private static final double kS = 0.057774;
    private static final double kG = 0.0;
    private static final double kV = 16.376;
    private static final double kA = 0.41226;

    private static double kDegUpperLimit = 30.0;
    private static double kDegLowerLimit = -25.0;
    private static final double kV = 0.0;
    private static final double kA = 0.0;

    private static double kDegUpperLimit = 30.0;
    private static double kDegLowerLimit = -25.0;
    
    private static final double kAngleIncrement = 2048;
    
    
    private static double kAngleIncrement;

    private static final double kEncoderResolution = 2048;
    
    private static ArmFeedforward m_feedforward;

    private static final double kStartAngle = -23.0;


    public ArmPIDandFeedForward() {
        this.talon = new WPI_TalonFX(5); // one instance of TalonSRX, replaced IntakeConstants.TALON_ID
        TalonFXConfiguration falconConfig = new TalonFXConfiguration();

        falconConfig.slot0.kP = 0;
        falconConfig.slot0.kI = 0;
        falconConfig.slot0.kD = 0;

        falconConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        falconConfig.voltageCompSaturation = 12;

        falconConfig.nominalOutputForward = 0;
        falconConfig.nominalOutputReverse = 0;
        falconConfig.peakOutputForward = 1;
        falconConfig.peakOutputReverse = -1;

        talon.configAllSettings(falconConfig);
        talon.setNeutralMode(NeutralMode.Coast);
        
        // kS and kG should have units of volts, kVk should have units of volts * seconds / radians, and kA should have units of volts * seconds^2 / radians.
        // Units must be consistent! TODO: get feedfowrward gains tuned
      
        m_feedforward = new ArmFeedforward(kS, kG, kV, kA);
        
        kAngleIncrement = (360 / kEncoderResolution) / 36;
        
        
    }

    // Singleton class, call getInstance to access instead of the constructor.
    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    public void setAngle(double precent) {

        double currentEncoder = talon.getSensorCollection().getIntegratedSensorPosition();
        System.out.println(currentEncoder);
        
        double pos = convertDegToRad(kStartAngle + (currentEncoder * kAngleIncrement))

        System.out.println("Goal angle:" + pos);
        
        // second arg is in rad/sec
        double feedOut = m_feedforward.calculate(pos, 0);

        if (pos >= kDegUpperLimit && feedOut > 0.0) {
            feedOut = 0.0;
        }
        else if (pos <= kDegLowerLimit && feedOut < 0.0) {
            feedOut = 0;
        }


        System.out.println("feedout: " + feedOut);

        talon.setVoltage(feedOut);
    }
    
    public void reset(){
        talon.getSensorCollection().setIntegratedSensorPosition(0, 30);
    }

    public double convertRadToDeg(double rad){
        return rad * (180 / Math.PI);
    }
    

}