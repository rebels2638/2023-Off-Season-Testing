// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swervedrive.module;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    //public double drivePositionRad = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveSetpointMetersPerSec = 0.0;
    //public double driveAppliedVolts = 0.0;
    //public double[] driveCurrentAmps = new double[] {};
    //public double[] driveTempCelcius = new double[] {};

    //public double turnAbsolutePositionRad = 0.0;
    public double turnPositionRad = 0.0;
    public double trunPositionDeg = 0.0;
    public double turnSetpointRad = 0.0;
    public double turnSetpointDeg = 0.0;
    
    // public double turnVelocityRadPerSec = 0.0;
    //public double turnAppliedVolts = 0.0;
    //public double[] turnCurrentAmps = new double[] {};
    //public double[] turnTempCelcius = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs, int moduleNumber) {}
}
