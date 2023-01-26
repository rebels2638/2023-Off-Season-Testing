package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FourBarArm extends SubsystemBase {
    private final WPI_TalonFX victor;
    private final CanSparkMax spark;
    
    private static FourBarArm instance = null;

    public FourBarArm() {
        this.talon = new WPI_TalonFX(0); // one instance of TalonSRX, replaced IntakeConstants.TALON_ID
        this.spark = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    // Singleton class, call getInstance to access instead of the constructor.
    public static FourBarArm getInstance() {
        if (instance == null) {
            instance = new FourBarArm();
        }
        return instance;
    }

    public void setPercentOutputVictor(double percent) {

        victor.set(ControlMode.PercentOutput, percent); // set talon speed based on input from XboxController.getleftY(), ie the input range on left y should map to the speed???? where speed is in range -1,1 and the xbox controller left joy stick is also -1,1???
    }
    
    public void setSpeedSpark(double speed) {
        
        spark.set(speed);
        
    }
    
}
