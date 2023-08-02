package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.json.*;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

public class JsonChanger extends CommandBase {

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

    public JsonChanger() throws IOException {
        String jsonContent = new String(Files.readAllBytes(Paths.get("/src/main/deploy/swerve/falcon/modules/pidfproperties.json")));
        JSONObject json = new JSONObject(jsonContent);

        drive = PIDF.fromJSON(json.getJSONObject("drive"));
        angle = PIDF.fromJSON(json.getJSONObject("angle"));

        System.out.println("Drive: P = " + drive.p + ", I = " + drive.i + ", D = " + drive.d + ", F = " + drive.f + ", Iz = " + drive.iz);
        System.out.println("Angle: P = " + angle.p + ", I = " + angle.i + ", D = " + angle.d + ", F = " + angle.f + ", Iz = " + angle.iz);

        // Modifying some values
        drive.p = 0.05;
        angle.f = 0.005;

        
    }
    @Override
    public void execute() {


        // Writing back to JSON
        JSONObject outputJson = new JSONObject();
        outputJson.put("drive", drive.toJSON());
        outputJson.put("angle", angle.toJSON());

        // Write JSON to file
        try (FileWriter file = new FileWriter("file.json")) {
            file.write(outputJson.toString());
            System.out.println("Successfully Copied JSON Object to File...");
        }
        
    }
    

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
