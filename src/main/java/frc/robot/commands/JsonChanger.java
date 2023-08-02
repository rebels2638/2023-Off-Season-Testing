package frc.robot.commands;

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

    public static class PIDF {
        public double p;
        public double i;
        public double d;
        public double f;
        public double iz;

        public PIDF(double p, double i, double d, double f, double iz) {
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
            this.iz = iz;
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

        public static PIDF fromJSON(JSONObject obj) {
            return new PIDF(
                obj.getDouble("p"),
                obj.getDouble("i"),
                obj.getDouble("d"),
                obj.getDouble("f"),
                obj.getDouble("iz")
            );
        }
    }

    private PIDF drive;
    private PIDF angle;
    private ShuffleboardTab tab = Shuffleboard.getTab("SwerveJson");
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
        String jsonContent = new String(Files.readAllBytes(Paths.get("src/main/deploy/swerve/falcon/modules/pidfproperties.json")));
        JSONObject json = new JSONObject(jsonContent);

        drive = PIDF.fromJSON(json.getJSONObject("drive"));
        angle = PIDF.fromJSON(json.getJSONObject("angle"));

        System.out.println("Drive: P = " + drive.p + ", I = " + drive.i + ", D = " + drive.d + ", F = " + drive.f + ", Iz = " + drive.iz);
        System.out.println("Angle: P = " + angle.p + ", I = " + angle.i + ", D = " + angle.d + ", F = " + angle.f + ", Iz = " + angle.iz);

        //do the rest for all pid values
        driveP = tab.add("driveP", drive.p).getEntry();
        driveI = tab.add("driveI", drive.i).getEntry();
        driveD = tab.add("driveD", drive.d).getEntry();
        driveF = tab.add("driveF", drive.f).getEntry();
        driveIz = tab.add("driveIz", drive.iz).getEntry();
        angleP = tab.add("angleP", angle.p).getEntry();
        angleI = tab.add("angleI", angle.i).getEntry();
        angleD = tab.add("angleD", angle.d).getEntry();
        angleF = tab.add("angleF", angle.f).getEntry();
        angleIz = tab.add("angleIz", angle.iz).getEntry();

        
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
        try (FileWriter file = new FileWriter("src/main/deploy/swerve/falcon/modules/pidfproperties.json")) {
            file.write(outputJson.toString());
            System.out.println("Successfully Copied JSON Object to File...");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
