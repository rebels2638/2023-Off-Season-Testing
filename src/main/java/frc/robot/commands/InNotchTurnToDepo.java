package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconDrivetrain;
import frc.robot.subsystems.PoseEstimator;

public class InNotchTurnToDepo extends CommandBase{
    FalconDrivetrain drive;
    PoseEstimator poseEstim;
    boolean leftOright;
    public InNotchTurnToDepo(FalconDrivetrain subsys, PoseEstimator subsys2){
        drive = subsys;
        poseEstim = subsys2;
        addRequirements(subsys);
        addRequirements(subsys2);
    }
    public void initialize(){
        double rot = poseEstim.getYaw(); 
        double destinRot = (Math.atan(poseEstim.getCurrentPose().getY()/poseEstim.getCurrentPose().getX()));
        drive.drive(0,(leftOright ? - destinRot :  destinRot) + (rot == 0 ? 0 : rot > 0 ? -rot : rot));
        
    }
    public void execute(){

    }
}