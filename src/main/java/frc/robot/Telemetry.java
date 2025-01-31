package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

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
        driveModulePositions.update(state.ModulePositions);
        driveTimestamp.update(state.Timestamp);
        driveOdometryFrequency.update(1.0 / state.OdometryPeriod);
    }
}
