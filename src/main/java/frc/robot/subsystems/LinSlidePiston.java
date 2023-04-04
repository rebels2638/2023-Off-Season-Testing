package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinSlidePiston extends SubsystemBase {

    private static LinSlidePiston instance = null;

    private final DoubleSolenoid solenoid;
    public boolean state; // push is true, and pull is false

    public LinSlidePiston() {
        this.solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
        this.push();
        state = true;
    }

    // Singleton class, call getInstance to access instead of the constructor.
    public static LinSlidePiston getInstance() {
        if (instance == null) {
            instance = new LinSlidePiston();
        }
        return instance;
    }

    public void push() {
        //System.out.println("here");
        solenoid.set(DoubleSolenoid.Value.kReverse);
        state = true;
    }

    public void pull() {
        solenoid.set(DoubleSolenoid.Value.kForward);
        state = false;
    }

    public void toggle() {
        if (state) {
            pull();
            return;
        } else {
            push();
            return;
        }
    }
}