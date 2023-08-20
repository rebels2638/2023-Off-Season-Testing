package frc.robot.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.swervelib.SwerveDrive;
import frc.lib.swervelib.telemetry.SwerveDriveTelemetry;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.Constants;

// acts more like a helper class rather than a subsystem or command.
public class AutoRunner {


    private final SendableChooser<String> pathChooser = new SendableChooser<String>();
    private String pathChosen;
    private static final HashMap<String, String> PATH_CHOSEN_TO_NAME_HASH_MAP = new HashMap<>();
    private static final HashMap<String, Command> EVENT_MAP = new HashMap<>();

    static {
        PATH_CHOSEN_TO_NAME_HASH_MAP.put("taxi", "taxi");
    }

    private SwerveSubsystem swerveSubsystem;
    public AutoRunner ( SwerveSubsystem swerveSubsystem ) {
        this.swerveSubsystem = swerveSubsystem;

        PATH_CHOSEN_TO_NAME_HASH_MAP.forEach((pathName, pathFile) -> pathChooser.addOption(pathName, pathFile));

        Shuffleboard.getTab("Auto").add("Path Chooser", pathChooser);
        Shuffleboard.getTab("Auto").add("Update Selected Command Output", 
            new InstantCommand( () -> loadPath()));
    }

    private void loadPath() {
        pathChosen = pathChooser.getSelected();
        Shuffleboard.getTab("Auto").add("Selected Path", pathChosen);
    }

    public Command getAutonomousCommand() {
        return swerveSubsystem.creatPathPlannerCommand
            ("OneAndBack1Working", 
            new PathConstraints(Constants.Auton.MAX_SPEED, 
            Constants.Auton.MAX_ACCELERATION), EVENT_MAP, 
            Constants.Auton.TRANSLATION_PID_CONFIG, 
            Constants.Auton.ANGLE_PID_CONFIG, true);
    }
} 