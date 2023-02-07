package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.motorcontrol.can.WPI_VicptorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; 
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FourBarArm extends SubsystemBase {
    private final WPI_TalonSRX talon;
    private final WPI_TalonSRX motor_775;
    //private final WPI_VictorSPX victor;
    // private final CANSparkMax spark;
    
    private static FourBarArm instance = null;

    public FourBarArm() {
        //this.victor = new WPI_VictorSPX(5);
        this.talon = new WPI_TalonSRX(7); // one instance of TalonSRX, replaced IntakeConstants.TALON_ID
        // this.spark = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        talon.setNeutralMode(NeutralMode.Brake);
        this.motor_775 = new WPI_TalonSRX(8); // change when found
        motor_775.setNeutralMode(NeutralMode.Brake);
        // spark.setInverted(true);
    }

    // Singleton class, call getInstance to access instead of the constructor.
    public static FourBarArm getInstance() {
        if (instance == null) {
            instance = new FourBarArm();
        }
        return instance;
    }

    public void setPercentOutputTalon(double percent) {

        talon.set(ControlMode.PercentOutput, percent); // set talon speed based on input from XboxController.getleftY(), ie the input range on left y should map to the speed???? where speed is in range -1,1 and the xbox controller left joy stick is also -1,1???
    }

    public void setPercentOutput775(double percent) {

        motor_775.set(ControlMode.PercentOutput, percent);
    }
    
    // public void setSpeedSpark(double speed) {
        
    //     spark.set(speed);
        
    // }
    
}
