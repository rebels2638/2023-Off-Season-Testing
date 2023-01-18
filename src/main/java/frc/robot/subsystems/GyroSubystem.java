package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix.Util;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS; 

public class GyroSubystem {
    private final static AHRS gyro = new AHRS(SPI.Port.kMXP);
    private static double offsetAngle = 0;
    private static double gyroReverser = 1;
    private static MedianFilter vxFilter = new MedianFilter(5);
    public GyroSubystem(){
        offsetAngle = 0;
    }   
    public static double getGyroYaw() { //yaw is rotation (turning) left or right
        //negative because trajectory requires counterclockwise rotation to be positive
        double rawAngle = getAccumulatedAngle();
        double angle = (rawAngle + 180.0) % 360.0;
        if (angle < 0) {
        angle += 360;
        }
        return angle - 180;
    }

    public static double getAccumulatedAngle() { 
        return (-1.034818 * gyro.getAngle()) + offsetAngle;
    }
    public static void setAngleOffset(double offsetDeg) {
        offsetAngle = offsetDeg;
    }
    public static double getGyroGlobalVx() {
        return vxFilter.calculate(-gyro.getVelocityX());
      }
    
      public static double getGyroGlobalVy() {
        return gyro.getVelocityY();
      }
    
      public static double getGyroAccelX(){
        return gyro.getWorldLinearAccelX();
      }
    
      public static double getGyroAccelY(){
        return gyro.getWorldLinearAccelY();
      }
    
      public static double getGyroAngV() {
        //return Units.radiansToDegrees(-gyro.getRate());
        return -gyro.getRate() * gyro.getActualUpdateRate();
      }
    
      public static void zeroGyroYaw() {
        gyro.zeroYaw();
        //gyro.setAngleAdjustment(adjustment);
      }
    
      public static boolean gyroIsCalibrating() {
        return gyro.isCalibrating();
      }
    
      public static void reverseGyro() {
        gyroReverser = -1;
      }
    
      public static void unReverseGyro() {
        gyroReverser = 1;
      }

}
