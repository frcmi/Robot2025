package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.ultralogger.*;

public class Telemetry {
    /* Robot swerve drive state */
    private final static String swervePrefix = "Swerve/";
    private final UltraStructLog<Pose2d> drivePose = new UltraStructLog<>(swervePrefix + "Pose", Pose2d.struct);
    private final UltraStructLog<ChassisSpeeds> driveSpeeds = new UltraStructLog<>(swervePrefix + "Speeds", ChassisSpeeds.struct);
    private final UltraStructArrayLog<SwerveModuleState> driveModuleStates = new UltraStructArrayLog<>(swervePrefix + "ModuleStates", SwerveModuleState.struct);
    private final UltraStructArrayLog<SwerveModuleState> driveModuleTargets = new UltraStructArrayLog<>(swervePrefix + "ModuleTargets", SwerveModuleState.struct);
    private final UltraStructArrayLog<SwerveModulePosition> driveModulePositions = new UltraStructArrayLog<>(swervePrefix + "ModulePositions", SwerveModulePosition.struct);
    private final UltraDoubleLog driveTimestamp = new UltraDoubleLog(swervePrefix + "Timestamp");
    private final UltraDoubleLog driveOdometryFrequency = new UltraDoubleLog(swervePrefix + "OdometryFrequency");


    private final UltraStructArrayLog<SwerveModuleState> azimuthNormalizedStates = new UltraStructArrayLog<>(swervePrefix + "AzimuthNormalizedStates", SwerveModuleState.struct);
    private final UltraStructArrayLog<SwerveModuleState> azimuthNormalizedTargets = new UltraStructArrayLog<>(swervePrefix + "AzimuthNormalizedTargets", SwerveModuleState.struct);
    
    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry() {
        SignalLogger.start();
    }

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.update(state.Pose);
        driveSpeeds.update(state.Speeds);
        driveModuleStates.update(state.ModuleStates);
        driveModuleTargets.update(state.ModuleTargets);
        azimuthNormalizedStates.update(normalizeAngles(state.ModuleStates));
        azimuthNormalizedTargets.update(normalizeAngles(state.ModuleTargets));
        driveModulePositions.update(state.ModulePositions);
        driveTimestamp.update(state.Timestamp);
        driveOdometryFrequency.update(1.0 / state.OdometryPeriod);
    }

    public SwerveModuleState[] normalizeAngles(SwerveModuleState[] states) {
        SwerveModuleState[] newStates = states.clone();
        for (int i = 0; i < newStates.length; i++) {
            SwerveModuleState state = newStates[i];
            double angle = state.angle.getDegrees();
            if (state.speedMetersPerSecond < 0) {
                angle += 180;
                newStates[i].speedMetersPerSecond *= -1;
            }
            newStates[i].angle = Rotation2d.fromDegrees((angle % 360 + 360) % 180);
        }

        return newStates;
    }
}
