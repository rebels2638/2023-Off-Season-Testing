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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorDownLinSlideIn;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.ElevatorUpLinSlideOut;
import frc.robot.commands.LinSlideFullyIn;
import frc.robot.commands.LinSlideFullyOut;
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
import java.util.HashMap;
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
        PATH_COMMANDS.put("clawOpen", new InstantCommand(Claw.getInstance()::push));
        PATH_COMMANDS.put("clawClose", new InstantCommand(Claw.getInstance()::pull));
        PATH_COMMANDS.put("resetDTEncoders", new InstantCommand(FalconDrivetrain.getInstance()::zeroEncoder));
        PATH_COMMANDS.put("elevatorFullUp", new ElevatorUp(ElevatorPIDNonProfiled.getInstance()));
        PATH_COMMANDS.put("elevatorFullDown", new ElevatorDown(ElevatorPIDNonProfiled.getInstance()));
        PATH_COMMANDS.put("elevatorUpLinSlideOut", new ElevatorUpLinSlideOut());
        PATH_COMMANDS.put("elevatorDownLinSlideIn", new ElevatorDownLinSlideIn());

        // idk if this should be dynamically loaded, as in using java.io.*
        PATHS.put("testPath", "testPath");
    }

    private FalconDrivetrain m_drive;
    private boolean finished = false;
    private ShuffleboardTab tab = Shuffleboard.getTab("Auto");
    public static PathPlannerTrajectory m_path;

    public static AutoRunner instance = null;
    private RamseteAutoBuilder m_autoBuilder;
    private Command m_autoCommand;

    private final SendableChooser<String> pathChooser = new SendableChooser<String>();

    private PoseEstimator m_poseEstimator;

    public AutoRunner() {
        m_drive = FalconDrivetrain.getInstance();
        m_poseEstimator = PoseEstimator.getInstance();
        m_autoBuilder = new RamseteAutoBuilder(
                m_drive::getPose,
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

    public PathPlannerTrajectory getPath() {
        return m_path;
    }

    public void loadPath() {
        m_path = PathPlanner.loadPath(pathChooser.getSelected(), new PathConstraints(2, 0.25));
        System.out.println(pathChooser.getSelected());
    }

    public Command getCommand() {
        loadPath();
        return m_autoBuilder.fullAuto(m_path);
    }
}
