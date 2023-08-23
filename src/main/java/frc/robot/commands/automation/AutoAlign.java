package frc.robot.commands.automation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.Constants;

// helper class
public class AutoAlign extends CommandBase {
    private SwerveSubsystem swerveSubsystem;
    private int targetNum;
    public AutoAlign (SwerveSubsystem swerveSubsystem, int targetNum) {
        this.targetNum = targetNum;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    private Command align( int targetNum ) {
        ArrayList<PathPoint> pathPoints = new ArrayList<PathPoint>();

        Translation2d currentTranslation = swerveSubsystem.getPose().getTranslation();
        Translation2d targetTranslation = Constants.FeildConstants.autoAlignTranslationArr[targetNum];


        Rotation2d angleToTarget;
        if (currentTranslation.getX() - targetTranslation.getX() > .5 && currentTranslation.getX() - targetTranslation.getX() > 1) {
            angleToTarget = new Rotation2d(Math.toRadians(180));
        }
        else if (currentTranslation.getY() > targetTranslation.getY() +1.5) {
            angleToTarget = new Rotation2d(Math.toRadians(270));
        }
        else {
            angleToTarget = new Rotation2d(Math.toRadians(90));
        }
        SmartDashboard.putNumber("swerve/angleToTarget", angleToTarget.getDegrees());
        pathPoints.add(new PathPoint(currentTranslation,
                angleToTarget,
                new Rotation2d(Math.toRadians(0)),
                (Math.abs(swerveSubsystem.getRobotVelocity().vxMetersPerSecond) + 
                  Math.abs(swerveSubsystem.getRobotVelocity().vyMetersPerSecond))));

        SmartDashboard.putNumber("swerve/totalVelocity", Math.abs(swerveSubsystem.getRobotVelocity().vxMetersPerSecond) + 
        Math.abs(swerveSubsystem.getRobotVelocity().vyMetersPerSecond));
        // end goal
        pathPoints.add( new PathPoint(targetTranslation, 
        angleToTarget,
        new Rotation2d(Math.toRadians(0)), 0));

        double[] log = { swerveSubsystem.getPose().getTranslation().getX(), swerveSubsystem.getPose().getTranslation().getY() };
        SmartDashboard.putNumberArray("swerve/align", log);

        PathPlannerTrajectory trajectory = PathPlanner.generatePath( 
            new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION), pathPoints);

        PIDController translationController = new PIDController(Constants.Auton.TRANSLATION_PID_CONFIG.kP,
                                                            Constants.Auton.TRANSLATION_PID_CONFIG.kI,
                                                            Constants.Auton.TRANSLATION_PID_CONFIG.kD);
                                                            
        PIDController rotationController = new PIDController(Constants.Auton.ANGLE_PID_CONFIG.kP,
                                                            Constants.Auton.ANGLE_PID_CONFIG.kI,
                                                            Constants.Auton.ANGLE_PID_CONFIG.kD);

        PPSwerveControllerCommand pathCommand = new PPSwerveControllerCommand(trajectory, 
                                                    swerveSubsystem::getPose,
                                                    translationController, 
                                                    translationController, 
                                                    rotationController, 
                                                    swerveSubsystem::setChassisSpeeds, swerveSubsystem);

        SmartDashboard.putNumber("swerve/poseLog", swerveSubsystem.getPose().getTranslation().getX());               
        return pathCommand;
    }

    @Override
    public void initialize() {
        Command pathCommand = align(targetNum);
        CommandScheduler.getInstance().schedule(pathCommand);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
