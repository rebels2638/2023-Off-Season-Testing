package frc.robot.auto;

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
public class AutoAlign {
    private SwerveSubsystem swerveSubsystem;
    public AutoAlign (SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    public Command align( int targetNum ) {
        ArrayList<PathPoint> pathPoints = new ArrayList<PathPoint>();
        Translation2d currentTranslation = swerveSubsystem.getPose().getTranslation();
        // current robot state 
        pathPoints.add(new PathPoint(currentTranslation,
                 swerveSubsystem.getHeading(),
                  swerveSubsystem.getPitch(),
                  (swerveSubsystem.getRobotVelocity().vxMetersPerSecond + swerveSubsystem.getRobotVelocity().vyMetersPerSecond)) );
        
        // end goal
        pathPoints.add( new PathPoint(Constants.FeildConstants.autoAlignTranslationArr[targetNum], 
        new Rotation2d(Math.toRadians(180)),
        new Rotation2d(Math.toRadians(0))));

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
}
