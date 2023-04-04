// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

// public class Camera extends SubsystemBase {
//   /** Creates a new ExampleSubsystem. */

//   private PhotonCamera photonCamera;
//   public Camera() {
//      photonCamera = new PhotonCamera("camera");
//   }

//   /**
//    * Example command factory method.
//    *

//   /**
//    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
//    *
//    * @return value of some boolean subsystem state, such as a digital sensor.
//    */
//   public boolean exampleCondition() {
//     // Query some boolean state, such as a digital sensor.
//     return false;
//   }

//   @Override
//   public void periodic() {
//     Shuffleboard.getTab("Camera").add("Elevator View",
    
    
//     photonCamera.getLatestResult());
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }
// }