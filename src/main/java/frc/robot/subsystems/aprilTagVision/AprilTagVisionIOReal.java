// package frc.robot.subsystems.aprilTagVision;

// import org.photonvision.PhotonCamera;

// import edu.wpi.first.math.geometry.Pose3d;
// import frc.robot.utils.Constants;

// public class AprilTagVisionIOReal implements AprilTagVisionIO {
//     private PhotonCamera frontRightCamera;
//     private PhotonCamera frontLeftCamera;
//     private PhotonCamera backRightCamera;
//     private PhotonCamera backLeftCamera;

//     public AprilTagVisionIOReal() {
//         frontRightCamera = new PhotonCamera(Constants.VisionConstants.FRONT_RIGHT_CAMERA_NAME);
//         frontLeftCamera = new PhotonCamera(Constants.VisionConstants.FRONT_LEFT_CAMERA_NAME);
//         backRightCamera = new PhotonCamera(Constants.VisionConstants.BACK_RIGHT_CAMERA_NAME);
//         backLeftCamera = new PhotonCamera(Constants.VisionConstants.BACK_LEFT_CAMERA_NAME);
//     }
    
//     @Override
//     public void updateInputs(AprilTagVisionIOInputs inputs, Pose3d refrencePose) {
//         inputs.frontRightBestTarget = frontRightCamera.getLatestResult().getBestTarget();
//         inputs.frontLeftBestTarget = frontLeftCamera.getLatestResult().getBestTarget();
//         inputs.backRightBestTarget = backRightCamera.getLatestResult().getBestTarget();
//         inputs.backLeftBestTarget = backLeftCamera.getLatestResult().getBestTarget();

//         inputs.frontRightPipleineLatency = frontRightCamera.getLatestResult().getLatencyMillis();
//         inputs.frontLeftPipleineLatency = frontLeftCamera.getLatestResult().getLatencyMillis();
//         inputs.backRightPipleineLatency = backRightCamera.getLatestResult().getLatencyMillis();
//         inputs.backLeftPipleineLatency = backLeftCamera.getLatestResult().getLatencyMillis();
//     }
// }