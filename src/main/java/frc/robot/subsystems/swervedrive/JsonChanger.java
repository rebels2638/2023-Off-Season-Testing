package frc.robot.subsystems.swervedrive;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.json.*;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

public class JsonChanger extends SubsystemBase {

    public class PIDF {
        public double p = 0;
        public double i = 0;
        public double d = 0;
        public double f = 0;
        public double iz = 0;

        public PIDF(double p, double i, double d, double f, double iz) {
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
            this.iz = iz;
            System.out.println("PIDFPFIFFASKFN");
        }

        public JSONObject toJSON() {
            JSONObject obj = new JSONObject();
            obj.put("p", p);
            obj.put("i", i);
            obj.put("d", d);
            obj.put("f", f);
            obj.put("iz", iz);
            return obj;
        }

        public PIDF fromJSON(JSONObject obj) {
            return new PIDF(
                obj.getDouble("p"),
                obj.getDouble("i"),
                obj.getDouble("d"),
                obj.getDouble("f"),
                obj.getDouble("iz")
            );
        }
    }

    private PIDF drive = new PIDF(0, 0, 0, 0, 0);
    private PIDF angle = new PIDF(0, 0, 0, 0, 0);
    private ShuffleboardTab tab;
    private GenericEntry driveP;
    private GenericEntry driveI;
    private GenericEntry driveD;
    private GenericEntry driveF;
    private GenericEntry driveIz;

    private GenericEntry angleP;
    private GenericEntry angleI;
    private GenericEntry angleD;
    private GenericEntry angleF;
    private GenericEntry angleIz;

    public JsonChanger() throws IOException {
        String jsonContent = new String(Files.readAllBytes(Paths.get("/src/main/deploy/swerve/falcon/modules/pidfproperties.json")));
        JSONObject json = new JSONObject(jsonContent);
        JSONObject driveJSON = json.getJSONObject("drive");
        JSONObject angleJSON = json.getJSONObject("angle");

        drive.fromJSON(driveJSON);
        drive.fromJSON(angleJSON);


        // System.out.println("Drive: P = " + drive.p + ", I = " + drive.i + ", D = " + drive.d + ", F = " + drive.f + ", Iz = " + drive.iz);
        // System.out.println("Angle: P = " + angle.p + ", I = " + angle.i + ", D = " + angle.d + ", F = " + angle.f + ", Iz = " + angle.iz);

        tab = Shuffleboard.getTab("SwerveJson");
        //do the rest for all pid values
        driveP = tab.add("driveP", 0).getEntry();
        driveI = tab.add("driveI", driveI).getEntry();
        driveD = tab.add("driveD", driveD).getEntry();
        driveF = tab.add("driveF", driveF).getEntry();
        driveIz = tab.add("driveIz", driveIz).getEntry();
        angleP = tab.add("angleP", angleP).getEntry();
        angleI = tab.add("angleI", angleI).getEntry();
        angleD = tab.add("angleD", angleD).getEntry();
        angleF = tab.add("angleF", angleF).getEntry();
        angleIz = tab.add("angleIz", angleIz).getEntry();

        
    }
    @Override
    public void periodic() {

        // Updating the values on Shuffleboard
        driveP.setDouble(drive.p);
        driveI.setDouble(drive.i);
        driveD.setDouble(drive.d);
        driveF.setDouble(drive.f);
        driveIz.setDouble(drive.iz);
        angleP.setDouble(angle.p);
        angleI.setDouble(angle.i);
        angleD.setDouble(angle.d);
        angleF.setDouble(angle.f);
        angleIz.setDouble(angle.iz);

        // Writing back to JSON
        JSONObject outputJson = new JSONObject();
        outputJson.put("drive", drive.toJSON());
        outputJson.put("angle", angle.toJSON());

        // Write JSON to file
        try (FileWriter file = new FileWriter("/src/main/deploy/swerve/falcon/modules/pidfproperties.json")) {
            file.write(outputJson.toString());
            System.out.println("Successfully Copied JSON Object to File...");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
}
// ********** Robot program startup complete **********
// Unhandled exception: java.lang.NullPointerException: Cannot read field "p" because "this.drive" is null