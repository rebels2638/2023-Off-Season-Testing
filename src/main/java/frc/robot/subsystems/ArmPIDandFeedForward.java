package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller;

public class ArmPIDandFeedForward extends SubsystemBase {
    private final WPI_TalonFX talon;
    private static Arm instance = null;
    private static double lastPercentSpeed; 
  
  
    private static final double kS;
    private static final double kG;
    private static final double kV;
    private static final double kA;
    private static final ArmFeedforward feedforward;
    
    

    public ArmPIDandFeedForward() {
        this.talon = new WPI_TalonFX(5); // one instance of TalonSRX, replaced IntakeConstants.TALON_ID
        lastPercentSpeed = 0;
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
      
        feedforward = new Arm
        
    }

    // Singleton class, call getInstance to access instead of the constructor.
    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    public void setPercentOutput(double percent) {
        
        if(Math.abs(percent) < 0.08) {
            percent = 0;
            //talon.enableBrakeMode(true); 
        }
      
        if(percent == 0) {
            //talon.enableBrakeMode(true);    
        }
        // System.out.println(percent);

        talon.set(ControlMode.PercentOutput, percent); 
        // set talon speed based on input from XboxController.getleftY(), ie the input range on left y should map to the speed, where both the speed and the left joy stick is in range -1,1
    }
}
