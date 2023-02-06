package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final WPI_TalonFX talon;
    private final GenericEntry tab;
    private static Arm instance = null;
    private static double lastPercentSpeed; 
    private static double kUpperLimit = 55000.0;
    private static double kLowerLimit = -65000.0;

    public Arm() {
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
        talon.setNeutralMode(NeutralMode.Brake);
        
        tab = Shuffleboard.getTab("SmartDashboard").add("Arm Encoder", 0.0).getEntry();
    }

    // Singleton class, call getInstance to access instead of the constructor.
    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    public void setPercentOutput(double percent) {
        double currentEncoder = talon.getSensorCollection().getIntegratedSensorPosition();

        if (currentEncoder >= kUpperLimit && percent > 0.0) {
            percent = 0.0;
        }
        else if (currentEncoder <= kLowerLimit && percent < 0.0) {
            percent = 0;
        }

        talon.set(ControlMode.PercentOutput, percent); 
        // set talon speed based on input from XboxController.getleftY(), ie the input range on left y should map to the speed, where both the speed and the left joy stick is in range -1,1
    }

    @Override
    public void periodic() {
        tab.setDouble(talon.getSelectedSensorPosition());
    }

    public void zeroEncoder() {
        talon.getSensorCollection().setIntegratedSensorPosition(0, 30);
    }
}
