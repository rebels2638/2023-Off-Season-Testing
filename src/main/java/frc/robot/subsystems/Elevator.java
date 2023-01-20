package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final WPI_TalonFX victor;
    private static Elevator instance = null;
    private static double lastPercentSpeed; 

    public Elevator() {
        this.victor = new WPI_TalonFX(6); // one instance of TalonSRX, replaced IntakeConstants.TALON_ID
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

        victor.configAllSettings(falconConfig);
        victor.setNeutralMode(NeutralMode.Coast);
        
    }

    // Singleton class, call getInstance to access instead of the constructor.
    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

    public void setPercentOutput(double percent) {
        System.out.println(percent);
        
        // if (Math.abs(percent) < 0.1) {
        //     percent = lastPercentSpeed;
        //     System.out.println("PERCENT EXPECTED: " + lastPercentSpeed);
        // }
        
        // else {       
        //     lastPercentSpeed = percent;
        // }

        // if(percent < 0) {
        //     percent += lastPercentSpeed;
        // }
        
        victor.set(ControlMode.PercentOutput, percent); // set talon speed based on input from XboxController.getleftY(), ie the input range on left y should map to the speed???? where speed is in range -1,1 and the xbox controller left joy stick is also -1,1???
      
    }
}
