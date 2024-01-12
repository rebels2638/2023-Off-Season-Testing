package frc.robot.auto;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.swervelib.SwerveDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.Constants;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// acts more like a helper class rather than a subsystem or command.
public class AutoRunner {


    private final SendableChooser<String> pathChooser = new SendableChooser<String>();
    private String pathChosen = "taxi";
    private static final HashMap<String, String> PATH_CHOSEN_TO_NAME_HASH_MAP = new HashMap<>();
    private static final HashMap<String, Command> EVENT_MAP = new HashMap<>();
    private static PathPlanner pathPlanner = new PathPlanner();
    private static PPSwerveControllerCommand scc;
    private static HolonomicDriveController hdc = new HolonomicDriveController(
        new PIDController(0.1, 0, 0), new PIDController(.1, 0, 0), new ProfiledPIDController(0.1, 0, 0,new Constraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION)));
    private static List<PathPlannerTrajectory> pathList;
    private static PathPlannerTrajectory traj;

    static {
        PATH_CHOSEN_TO_NAME_HASH_MAP.put("taxi", "taxi");
        PATH_CHOSEN_TO_NAME_HASH_MAP.put("Turn Auto", "Turn Auto");
        PATH_CHOSEN_TO_NAME_HASH_MAP.put("Turn In Place","Turn In Place");
        PATH_CHOSEN_TO_NAME_HASH_MAP.put("Turn", "Turn");

        
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
        // return pathPlanner.loadPath(pathChooser.getSelected(), Constants.Auton.MAX_SPEED, 
        // Constants.Auton.MAX_ACCELERATION, false);
        traj = PathPlanner.fromPathFile(pathChosen);
        scc = new PPSwerveControllerCommand(traj, 
                                                    swerveSubsystem::getPose,
                                                    new PIDController(Constants.Auton.TRANSLATION_PID_CONFIG.kP,
                                                            Constants.Auton.TRANSLATION_PID_CONFIG.kI,
                                                            Constants.Auton.TRANSLATION_PID_CONFIG.kD), 
                                                    new PIDController(Constants.Auton.TRANSLATION_PID_CONFIG.kP,
                                                            Constants.Auton.TRANSLATION_PID_CONFIG.kI,
                                                            Constants.Auton.TRANSLATION_PID_CONFIG.kD), 
                                                    new PIDController(Constants.Auton.ANGLE_PID_CONFIG.kP,
                                                            Constants.Auton.ANGLE_PID_CONFIG.kI,
                                                            Constants.Auton.ANGLE_PID_CONFIG.kD), 
                                                    swerveSubsystem::setChassisSpeeds, swerveSubsystem);

         return scc;
        // return scc;
        
        return swerveSubsystem.creatPathPlannerCommand
            (pathChosen, 
            new PathConstraints(Constants.Auton.MAX_SPEED, 
            Constants.Auton.MAX_ACCELERATION), EVENT_MAP, 
            Constants.Auton.TRANSLATION_PID_CONFIG, 
            Constants.Auton.ANGLE_PID_CONFIG, true);
    }
} 