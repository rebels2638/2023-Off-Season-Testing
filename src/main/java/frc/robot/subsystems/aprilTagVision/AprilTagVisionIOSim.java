// package frc.robot.subsystems.aprilTagVision;

// import org.photonvision.PhotonCamera;
// import org.photonvision.SimVisionSystem;

// import org.photonvision.PhotonCamera;
// import org.photonvision.SimVisionSystem;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import frc.robot.utils.Constants;

// public class AprilTagVisionIOSim implements AprilTagVisionIO {
//      private PhotonCamera frontRightCamera;
//      private PhotonCamera frontLeftCamera;
//      private PhotonCamera backRightCamera;
//      private PhotonCamera backLeftCamera;

//      private SimVisionSystem frontRightVisionSystem;
//      private SimVisionSystem frontLeftVisionSystem;
//      private SimVisionSystem backRightVisionSystem;
//      private SimVisionSystem backLeftVisionSystem;
    
//      public AprilTagVisionIOSim() {
//          frontRightVisionSystem = new SimVisionSystem(
//             Constants.VisionConstants.FRONT_RIGHT_CAMERA_NAME, 
//             Constants.VisionConstants.FRONT_RIGHT_CAMERA_FOV_DIAGONAL, 
//             Constants.VisionConstants.FRONT_RIGHT_CAMERA_ROBOT_TO_CAMERA,
//             Constants.VisionConstants.FRONT_RIGHT_MAX_LED_RANGE_METERS, 
//             Constants.VisionConstants.FRONT_RIGHT_CAMERA_RESOLUTION_WIDTH,
//             Constants.VisionConstants.FRONT_RIGHT_CAMERA_RESOLUTION_HEIGHT,
//             Constants.VisionConstants.FRONT_RIGHT_CAMERA_MIN_TARGET_AREA);
//         frontLeftVisionSystem = new SimVisionSystem(
//             Constants.VisionConstants.FRONT_LEFT_CAMERA_NAME, 
//             Constants.VisionConstants.FRONT_LEFT_CAMERA_FOV_DIAGONAL, 
//             Constants.VisionConstants.FRONT_LEFT_CAMERA_ROBOT_TO_CAMERA,
//             Constants.VisionConstants.FRONT_LEFT_MAX_LED_RANGE_METERS, 
//             Constants.VisionConstants.FRONT_LEFT_CAMERA_RESOLUTION_WIDTH,
//             Constants.VisionConstants.FRONT_LEFT_CAMERA_RESOLUTION_HEIGHT,
//             Constants.VisionConstants.FRONT_LEFT_CAMERA_MIN_TARGET_AREA);
//         backRightVisionSystem = new SimVisionSystem(
//             Constants.VisionConstants.BACK_RIGHT_CAMERA_NAME, 
//             Constants.VisionConstants.BACK_RIGHT_CAMERA_FOV_DIAGONAL, 
//             Constants.VisionConstants.BACK_RIGHT_CAMERA_ROBOT_TO_CAMERA,
//             Constants.VisionConstants.BACK_RIGHT_MAX_LED_RANGE_METERS, 
//             Constants.VisionConstants.BACK_RIGHT_CAMERA_RESOLUTION_WIDTH,
//             Constants.VisionConstants.BACK_RIGHT_CAMERA_RESOLUTION_HEIGHT,
//             Constants.VisionConstants.BACK_RIGHT_CAMERA_MIN_TARGET_AREA);
//         backLeftVisionSystem = new SimVisionSystem(
//             Constants.VisionConstants.BACK_LEFT_CAMERA_NAME, 
//             Constants.VisionConstants.BACK_LEFT_CAMERA_FOV_DIAGONAL, 
//             Constants.VisionConstants.BACK_LEFT_CAMERA_ROBOT_TO_CAMERA,
//             Constants.VisionConstants.BACK_LEFT_MAX_LED_RANGE_METERS, 
//             Constants.VisionConstants.BACK_LEFT_CAMERA_RESOLUTION_WIDTH,
//             Constants.VisionConstants.BACK_LEFT_CAMERA_RESOLUTION_HEIGHT,
//             Constants.VisionConstants.BACK_LEFT_CAMERA_MIN_TARGET_AREA);
        
//         frontRightCamera = new PhotonCamera(Constants.VisionConstants.FRONT_RIGHT_CAMERA_NAME);
//         frontLeftCamera = new PhotonCamera(Constants.VisionConstants.FRONT_LEFT_CAMERA_NAME);
//         backRightCamera = new PhotonCamera(Constants.VisionConstants.BACK_RIGHT_CAMERA_NAME);
//         backLeftCamera = new PhotonCamera(Constants.VisionConstants.BACK_LEFT_CAMERA_NAME);
//     }

//     @Override
//     public void updateInputs(AprilTagVisionIOInputs inputs, Pose3d refrencePose) {
//         Pose2d robotPose = refrencePose.toPose2d();

//         frontRightVisionSystem.processFrame(robotPose);
//         frontLeftVisionSystem.processFrame(robotPose);
//         backRightVisionSystem.processFrame(robotPose);
//         backLeftVisionSystem.processFrame(robotPose);

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