package frc.robot.subsystems.swervedrive.module;
import frc.lib.swervelib.telemetry.SwerveDriveTelemetry;


public class ModuleIOYAGSL implements ModuleIO {
    public void updateInputs(ModuleIOInputs inputs, int moduleNumber) {
        inputs.driveVelocityMetersPerSec = SwerveDriveTelemetry.measuredStates[ (moduleNumber * 2) + 1 ];
        inputs.driveSetpointMetersPerSec = SwerveDriveTelemetry.desiredStates[ (moduleNumber * 2) + 1];

        inputs.trunPositionDeg = SwerveDriveTelemetry.measuredStates[moduleNumber * 2];
        inputs.turnPositionRad = Math.toRadians(SwerveDriveTelemetry.measuredStates[moduleNumber * 2]);
        inputs.turnSetpointDeg = SwerveDriveTelemetry.desiredStates[moduleNumber * 2];
        inputs.turnSetpointRad = Math.toRadians(SwerveDriveTelemetry.desiredStates[moduleNumber * 2]);
    }
}
