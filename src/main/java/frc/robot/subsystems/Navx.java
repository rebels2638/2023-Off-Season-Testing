// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Navx extends SubsystemBase {
    private static Navx instance = null;

    public AHRS m_gyro = new AHRS(I2C.Port.kMXP);
    // public AnalogGyro m_gyro = new AnalogGyro(1);
    private double gyroAdjustmentSim = 0.0;
    private double pitchOffset = 0.0;

    public Navx() {
        pitchOffset = m_gyro.getPitch();
        // pitchOffset = 0.0;
    }

    public static Navx getInstance() {
        if (instance == null) {
            instance = new Navx();
        }
        return instance;
    }

    public Rotation2d getRotation() {
        return m_gyro.getRotation2d();
    }

    // pitch in degrees
    public double getPitch() {
        return m_gyro.getPitch() - pitchOffset;
        // return 0.0;
    }

    public double getGyroAngle() {
        return m_gyro.getAngle();
        // return m_gyro.getAngle() + gyroAdjustmentSim;
    }

    public double getGyroAngleUnmodified() {
        return m_gyro.getAngle() - m_gyro.getAngleAdjustment();
        // return m_gyro.getAngle();
    }

    public void resetPitchOffset() {
        pitchOffset = m_gyro.getPitch();
        // pitchOffset = 0.0;
    }

    public void setYawAdjustment(double deg) {
        m_gyro.setAngleAdjustment(deg);
        // gyroAdjustmentSim = deg;
    }

    public static double degreesToRadians(int degrees) {
        return (degrees / 180.0) * Math.PI;
    }

    public void resetGyroToPose(Pose2d pose) {
        resetPitchOffset();
        setYawAdjustment(pose.getRotation().getDegrees() - getGyroAngleUnmodified());
    }

    public void resetHeading() {
        m_gyro.reset();
    }
}