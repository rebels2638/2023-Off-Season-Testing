package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

    private static Claw instance = null;
    private CANSparkMax m_motor;

    private final DoubleSolenoid solenoid;
    private boolean state; // push is true, and pull is false

    private PIDController pid = new PIDController(0.0001, 0.0, 0.00000001);

    public Claw() {
        // this.victor = new VictorSPX(0); // one instance of TalonSRX, replaced
        // IntakeConstants.TALON_ID
        this.m_motor = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 4);
        this.push();
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
        solenoid.set(DoubleSolenoid.Value.kReverse);
        state = true;
    }

    public void pull() {
        solenoid.set(DoubleSolenoid.Value.kForward);
        state = false;

    }

    public void toggle() {
        if (state) {
            System.out.println("got here");
            pull();
            return;
        } else {
            System.out.println("not here");
            push();
            return;
        }
    }

    @Override
    public void periodic() {
        System.out.println(m_motor.getEncoder().getVelocity());
            m_motor.set(0.2 + pid.calculate(m_motor.getEncoder().getVelocity(), 679));
    }
}
