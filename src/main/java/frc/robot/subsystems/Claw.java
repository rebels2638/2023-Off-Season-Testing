package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

    private static Claw instance = null; 
    
    private final DoubleSolenoid solenoid;
    private boolean state; // push is true, and pull is false

    public Claw() {
        // this.victor = new VictorSPX(0); // one instance of TalonSRX, replaced IntakeConstants.TALON_ID
        
        this.solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
        state = true;
    }

    // Singleton class, call getInstance to access instead of the constructor.
    public static Claw getInstance() {
        if (instance == null) {
            instance = new Claw();
        }
        return instance;
    }
  
    public void push() {
        // prevent duplicating run
        if (state) {
            return;
        }

        solenoid.set(DoubleSolenoid.Value.kReverse);
        state = true;
    }

    public void pull() {
        // prevent duplicating run
        if (!state) {
            return;
        }
      
        solenoid.set(DoubleSolenoid.Value.kForward);
        state = false;

    }

}
