package frc.robot.commands.automation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
    private IntSupplier targetNum;
    private Command pathCommand;
    public AutoAlign (SwerveSubsystem swerveSubsystem, IntSupplier targetNum) {
        SmartDashboard.putNumber("swerve/aligment/targetNum", targetNum.getAsInt());

        this.targetNum = targetNum;
        this.swerveSubsystem = swerveSubsystem;
        // dont need to add swerve sub as a requiremtn because it never uses it. 
        // otherwise, pathCommand cant execute
    }

    private Command align( int targetNum ) {
        ArrayList<PathPoint> pathPoints = new ArrayList<PathPoint>();

        Translation2d currentTranslation = swerveSubsystem.getPose().getTranslation();
        Translation2d targetTranslation = Constants.FeildConstants.autoAlignTranslationArr[targetNum];
        
        PathPoint startPoint = new PathPoint(currentTranslation, new Rotation2d(Math.atan2((targetTranslation.getY() - currentTranslation.getY()),(targetTranslation.getX() - currentTranslation.getX()))), swerveSubsystem.getYaw(), 
            swerveSubsystem.getFieldVelocity().vxMetersPerSecond + swerveSubsystem.getFieldVelocity().vyMetersPerSecond);
        pathPoints.add(startPoint);

        // align just infront of the target
        Translation2d interiorTranslation = new Translation2d(targetTranslation.getX() + .01, targetTranslation.getY());

        PathPoint interiorPathPoint = new PathPoint(interiorTranslation,  new Rotation2d(Math.PI));

        pathPoints.add(interiorPathPoint);

        // end goal
        PathPoint endPoint = new PathPoint(targetTranslation, 
        new Rotation2d(Math.toRadians(Math.PI)),
        new Rotation2d(Math.toRadians(Math.PI)));

        pathPoints.add(endPoint);

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

        return pathCommand;
    }

    @Override
    public void initialize() {
        pathCommand = align(targetNum.getAsInt());
        CommandScheduler.getInstance().schedule(pathCommand);
    }
    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }
}
