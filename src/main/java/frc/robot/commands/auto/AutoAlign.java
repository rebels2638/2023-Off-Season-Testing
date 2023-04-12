package frc.robot.commands.auto;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.utils.AutoConstants;
import frc.robot.utils.AutoConstants.LimelightConstants;

public class AutoAlign extends CommandBase {
    private PIDController rpid = new PIDController(7, 0, 0.05);
    private PIDController dpid = new PIDController(2, 0, 0.05);

    private FalconDrivetrain m_drive;
    private Limelight m_limelight;
    private PoseEstimator m_estimator;

    private PhotonTrackedTarget lastTarget = null;

    public AutoAlign(FalconDrivetrain drive, Limelight limelight, PoseEstimator estimator) {
        m_drive = drive;
        m_limelight = limelight;
        m_estimator = estimator;
    }

    @Override
    public void initialize() {
        m_limelight.setMode(LimelightConstants.REFLECTIVETAPE_PIPELINE);
        rpid.setSetpoint(0);
        dpid.setSetpoint(LimelightConstants.CAM_TO_ARM_DIST);
        lastTarget = null;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        PhotonTrackedTarget result = m_limelight.getLatestTarget();
        if (result != null)
            lastTarget = result;
        if(lastTarget == null) {
            m_drive.drive(0, 0);
            return;
        }

        // Add gyro pitch for added accuracy
        double distanceToTarget = PhotonUtils.calculateDistanceToTargetMeters(
                LimelightConstants.ROBOT_TO_CAM_HEIGHT,
                AutoConstants.CONE_TARGET_HEIGHT,
                LimelightConstants.ROBOT_TO_CAM_PITCH + m_estimator.getPitch(),
                Units.degreesToRadians(lastTarget.getPitch()));

        // Add an offset to account for putting the arm in line with the target, not the limelight
        // Essentially we pretend that the camera had its yaw offsetted from the arm, which we consider to be the forward direction
        double targetYaw = Units.degreesToRadians(lastTarget.getYaw()) - LimelightConstants.CAM_TO_ARM_YAW;

        // Negate because we control robot, not target
        m_drive.drive(-rpid.calculate(targetYaw), -dpid.calculate(distanceToTarget));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_limelight.setMode(LimelightConstants.DEFAULT_PIPELINE);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    /*
     * Trajectory planning
     * 
     * 1. make simple networktables interface with all the cone / cube placement
     * buttons
     * 2. find current pose, and the pose we want to go (1 ft away from the final
     * pose (striaght on)) and create trajectory w/ ramsette controller stuff
     * a. assign which notches for which poles
     * 3. if statement so autorun can only happen after going past charge station
     * pose (x coord)
     * 4. go to pose (1ft away)
     * 5. find new pose to go in (straight 1ft)
     * 6. use limelight reflective tape to align with the pole (if we can use
     * multiple pipelines) (or even gyro or pose)
     * 7. align within threshold of degrees (5 or smth) and then do place preset
     * (current + a piston drop after)
     * 
     */
}