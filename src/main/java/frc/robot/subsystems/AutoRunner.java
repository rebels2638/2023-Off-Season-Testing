// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoPlace;
import frc.robot.commands.CallAutoBalance;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorDownLinSlideIn;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.ElevatorUpLinSlideOut;
import frc.robot.commands.LinSlideFullyIn;
import frc.robot.commands.LinSlideFullyOut;
import frc.robot.commands.Place;
import frc.robot.commands.WristDown;
import frc.robot.commands.WristUp;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.ElevatorPIDNonProfiled;
import frc.robot.subsystems.FalconDrivetrain;
import frc.robot.subsystems.LinSlidePiston;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.utils.AutoConstants;
import frc.robot.utils.ConstantsFXDriveTrain.DriveConstants;

import java.util.Map;
import java.util.stream.Stream;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.util.HashMap;
import java.util.List;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;

import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

public final class AutoRunner extends SubsystemBase {
    public static Command autoCommand;

    public static final Map<String, Command> PATH_COMMANDS = new HashMap<>();
    public static final Map<String, String> PATHS = new HashMap<>();
    static {
        PATH_COMMANDS.put("linPistonClose",
                new LinSlideFullyIn(LinearSlide.getInstance(), LinSlidePiston.getInstance()));
        PATH_COMMANDS.put("linPistonOpen",
                new LinSlideFullyOut(LinearSlide.getInstance(), LinSlidePiston.getInstance()));
        PATH_COMMANDS.put("clawOpen", new Place());
        PATH_COMMANDS.put("clawClose", new InstantCommand(Claw.getInstance()::pull));
        PATH_COMMANDS.put("resetDTEncoders", new InstantCommand(FalconDrivetrain.getInstance()::zeroEncoder));
        PATH_COMMANDS.put("elevatorFullUp", new ElevatorUp(ElevatorPIDNonProfiled.getInstance()));
        PATH_COMMANDS.put("elevatorFullDown", new ElevatorDown(ElevatorPIDNonProfiled.getInstance()));
        PATH_COMMANDS.put("elevatorUpLinSlideOut", new ElevatorUpLinSlideOut());
        PATH_COMMANDS.put("elevatorDownLinSlideIn", new ElevatorDownLinSlideIn());
        PATH_COMMANDS.put("resetGyro", new InstantCommand(PoseEstimator.getInstance()::resetHeading));
        PATH_COMMANDS.put("autoPlace", new AutoPlace());
        PATH_COMMANDS.put("wristDown", new WristDown(Wrist.getInstance()));
        PATH_COMMANDS.put("clawUnpinch", new InstantCommand(Claw.getInstance()::push));
        PATH_COMMANDS.put("wristUp", new WristUp(Wrist.getInstance()));
        PATH_COMMANDS.put("autoBalance", new AutoBalance(FalconDrivetrain.getInstance(), PoseEstimator.getInstance()));

        // idk if this should be dynamically loaded, as in using java.io.*

        PATHS.put("taxi", "taxi");
        PATHS.put("OneConeAndPick1", "OneConeAndPick1");
        PATHS.put("OneCubeAndPick1", "OneCubeAndPick1");
        PATHS.put("OneConeAndPickAndBalance1", "OneConeAndPickAndBalance1");
        PATHS.put("OneConeAndBalance2", "OneConeAndBalance2");
        PATHS.put("OneCubeAndBalance2", "OneCubeAndBalance2");
        PATHS.put("OneConeAndPick3", "OneConeAndPick3");
        PATHS.put("OneCubeAndPick3", "OneCubeAndPick3");
        PATHS.put("OneCubeLowAndPick3", "OneCubeLowAndPick3");

        // IGNORE
        // PATHS.put("OneAndBack3Working", "OneAndBack3Working");
        // PATHS.put("OneAndBack1Working", "OneAndBack1Working");
        // PATHS.put("OneAndBack1", "OneAndBack1");
        // PATHS.put("OneAndBack2", "OneAndBack2");
        // PATHS.put("OneAndBack3", "OneAndBack3");
    }

    private FalconDrivetrain m_drive;
    private boolean finished = false;
    private ShuffleboardTab tab = Shuffleboard.getTab("Auto");
    public static List<PathPlannerTrajectory> m_path;

    public static AutoRunner instance = null;
    private RamseteAutoBuilder m_autoBuilder;
    private Command m_autoCommand;

    private final SendableChooser<String> pathChooser = new SendableChooser<String>();

    private PoseEstimator m_poseEstimator;

    public AutoRunner() {
        m_drive = FalconDrivetrain.getInstance();
        m_poseEstimator = PoseEstimator.getInstance();
        pathChooser.setDefaultOption("taxi", "taxi");
        m_autoBuilder = new RamseteAutoBuilder(
                m_poseEstimator::getCurrentPose,
                m_poseEstimator::resetPose,
                new RamseteController(),
                m_drive.m_kinematics,
                m_drive.m_feedforward,
                m_drive::getWheelSpeeds,
                new PIDConstants(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
                m_drive::setVoltageFromAuto,
                PATH_COMMANDS,
                true,
                m_drive);

        // do not uncomment this, we are not using pplib client and server
        // if you are to uncomment, change the server port along in the driver station
        // to sync up the correct ports for the path transfers (on the fly)
        // PathPlannerServer.startServer(5811);//Double check this

        PATHS.forEach((pathName, pathFile) -> pathChooser.addOption(pathName, pathFile));

        Shuffleboard.getTab("Auto").add("Path", pathChooser);
        Shuffleboard.getTab("Auto").add("Load Path", new InstantCommand(() -> loadPath()));

    }

    public static AutoRunner getInstance() {
        if (instance == null) {
            instance = new AutoRunner();
        }
        return instance;
    }

    public List<PathPlannerTrajectory> getPath() {
        return m_path;
    }

    public void loadPath() {
        loadPathString(pathChooser.getSelected());
        System.out.println(pathChooser.getSelected());
    }

    public void loadPathString(String pathName) {
        boolean isReversed = false;
        List<PathConstraints> constraints = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(
                new FileReader(
                        new File(Filesystem.getDeployDirectory(), "pathplanner/" + pathName + ".path")))) {
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                fileContentBuilder.append(line);
            }

            String fileContent = fileContentBuilder.toString();
            JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

            isReversed = (boolean) json.get("isReversed");

            JSONArray jsonWaypoints = (JSONArray) json.get("waypoints");

            for (int i = 0; i < jsonWaypoints.size() - 1; i++) {
                JSONObject waypoint1 = (JSONObject) jsonWaypoints.get(i);
                JSONObject waypoint2 = (JSONObject) jsonWaypoints.get(i + 1);
                double constraint1 = (double) waypoint1.get("velOverride");
                double constraint2 = (double) waypoint2.get("velOverride");
                if(constraint1 != -1.0 && constraint2 != 1.0) {
                    constraints.add(new PathConstraints(Math.max(constraint1, constraint2), 0.75));
                } else {
                    constraints.add(new PathConstraints(1.5, 0.75));
                }
            }
            
        } catch (Exception e) {
            e.printStackTrace();
        }

        m_path = PathPlanner.loadPathGroup(pathName, isReversed, constraints.get(0), constraints.subList(1, constraints.size()).toArray(PathConstraints[]::new));
    }

    public Command getCommand() {
        return m_autoBuilder.fullAuto(m_path);
    }
}
