
package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ChaseTag extends CommandBase {
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2); 
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
    
    private static final int TAG_TO_CHASE = 2;
    private static final Transform3d ROBOT_TO_CAMERA = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, Math.PI));
    private static final Transform3d TAG_TO_GOAL = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, Math.PI));
    
    private final PhotonCamera photonCamera;
    private final Drivetrain drivetrainSubsystem;
    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(3, 0, 0, OMEGA_CONSTRAINTS);

    private PhotonTrackedTarget lastTarget;

    public ChaseTag(PhotonCamera photonCamera, Drivetrain drivetrainSubsystem, Supplier<Pose2d> poseProvider){
        this.photonCamera = photonCamera;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.poseProvider = poseProvider;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput((-Math.PI), Math.PI);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){
        lastTarget = null;
        var robotPose = poseProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }
    @Override
    public void execute(){
        var robotPose2d = poseProvider.get();
        var robotPose = (new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0.0, new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians())));
        var photonRes = photonCamera.getLatestResult();
        if(photonRes.hasTargets()){
            var targetOpt = photonRes.getTargets().stream()
            .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
            .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= 0.2)
            .findFirst();
            if(targetOpt.isPresent()){
                var target = targetOpt.get();
                lastTarget = target;
                var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

                var camToTarget = target.getBestCameraToTarget();
                var targetPose = cameraPose.transformBy(camToTarget);
            
                var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                xController.setGoal(goalPose.getX());
                yController.setGoal(goalPose.getY());
                omegaController.setGoal(goalPose.getRotation().getRadians());

            
            }   
        }
        if(lastTarget == null){
            //drivetrainSubsystem.stop();
        }
        else{
            var xSpeed = xController.calculate(robotPose.getX());
            if(xController.atGoal()){
                xSpeed = 0;
            }
            var ySpeed = yController.calculate(robotPose2d.getY());
            if(yController.atGoal()){
                ySpeed = 0;
            }
            var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
            if(omegaController.atGoal()){
                omegaSpeed = 0;
            }

            drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
        }
    }

    @Override
    public void end(boolean interrupted){
        //drivetrainSubsystem.stop()
    }


}
