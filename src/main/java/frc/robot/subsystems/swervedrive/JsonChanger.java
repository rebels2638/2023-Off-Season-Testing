package frc.robot.subsystems.swervedrive;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.json.*;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

public class JsonChanger extends SubsystemBase {

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
        //System.err.println("jsonchanger");
        String filecontentraw = new String(Files.readAllBytes(Paths.get(Filesystem.getDeployDirectory()+ "/swerve/falcon/modules/pidfproperties.json")));
        JSONObject filecontentjson = new JSONObject(filecontentraw);
        JSONObject driveJSON = filecontentjson.getJSONObject("drive");
        JSONObject angleJSON = filecontentjson.getJSONObject("angle");

        this.drive = drive.fromJSON(driveJSON);
        this.angle = angle.fromJSON(angleJSON);

        //System.out.println("Drive: P = " + drive.p + ", I = " + drive.i + ", D = " + drive.d + ", F = " + drive.f + ", Iz = " + drive.iz);
        //System.out.println("Angle: P = " + angle.p +c ", I = " + angle.i + ", D = " + angle.d + ", F = " + angle.f + ", Iz = " + angle.iz);

        tab = Shuffleboard.getTab("SwerveJson");
        // Tabs init.
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

    }
    @Override
    public void periodic() {

        drive.p = driveP.getDouble(0.0);
        drive.i = driveI.getDouble(0.0);
        drive.d = driveD.getDouble(0.0);
        drive.f = driveF.getDouble(0.0);
        drive.iz = driveIz.getDouble(0.0);

        angle.p = angleP.getDouble(0.0);
        angle.i = angleI.getDouble(0.0);
        angle.d = angleD.getDouble(0.0);
        angle.f = angleF.getDouble(0.0);
        angle.iz = angleIz.getDouble(0.0);

        // Writing back to JSON
        JSONObject outputJson = new JSONObject();
        outputJson.put("drive", drive.toJSON());
        outputJson.put("angle", angle.toJSON());

        // Write JSON to file
        try (FileWriter file = new FileWriter(Filesystem.getDeployDirectory()+ "/swerve/falcon/modules/pidfproperties.json")) {
            file.write(outputJson.toString());
            System.err.println("Successfully Copied JSON Object to File...");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

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
            // System.out.println("PIDFPFIFFASKFN");
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
}
// ********** Robot program startup complete **********
// Unhandled exception: java.lang.NullPointerException: Cannot read field "p" because "this.drive" is null