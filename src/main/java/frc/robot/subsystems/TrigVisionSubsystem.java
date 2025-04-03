package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ultralogger.UltraStringLog;
import frc.lib.ultralogger.UltraStructLog;
import frc.robot.Constants.AutoConstants;
import frc.robot.vision.LimeLightAprilTag;
import frc.robot.vision.LimeLightAprilTag.TagInfo;
import frc.robot.subsystems.LEDSubsystem;

public final class TrigVisionSubsystem extends SubsystemBase {
    private LimeLightAprilTag bargeCamera;
    private LimeLightAprilTag reefCamera;
    private Optional<TagInfo> lastSeenBargeTag = Optional.empty();
    public Optional<TagInfo> lastSeenReefTag = Optional.empty();
    private Time timeSinceBargeTagSeen = Seconds.of(0);
    private Time timeSinceReefTagSeen = Seconds.of(0);
    private LEDSubsystem m_LedSubsystem;

    private final double bargeCameraHeightMeters = Inches.of(10.5).in(Meters);   // Height of the camera off the ground in meters
    private final double bargeTargetHeightMeters = Inches.of(73).in(Meters);     // Height of the AprilTag on the field in meters
    private final double bargeCameraAngleYDegrees = AutoConstants.cameraAngle.in(Degrees);    // Angle at which the camera is mounted
    
    private final double reefTagHeightMeters = Inches.of(8.75).in(Meters);
    private final double reefCameraHeightMeters = Inches.of(6.9).in(Meters);
    private final double reefCameraAngleZDegrees = 10;
    private final double reefCameraAngleYDegrees = 15;

    public TrigVisionSubsystem(LEDSubsystem m_LedSubsystem) {
        bargeCamera = new LimeLightAprilTag("limelight-barge");
        reefCamera = new LimeLightAprilTag("limelight");

        this.m_LedSubsystem = m_LedSubsystem;
    }

    LEDPattern seeingColor = LEDPattern.solid(new Color(255, 0, 255));
    LEDPattern seeingColorReef = LEDPattern.solid(new Color(255, 255, 255));
    LEDPattern alignedColor = LEDPattern.solid(new Color(0, 255, 0));
    // LEDPattern notSeeingColor = LEDPattern.rainbow(255, 128);

    public double isAlignedTimestamp = 0;

    public boolean isAligned() {
        return Math.abs(RobotController.getFPGATime() - isAlignedTimestamp) / 1e6 < 0.1;
    }

    UltraStructLog<Translation2d> bargePosePublisher = new UltraStructLog<>("Vision/Barge Offset", Translation2d.struct);
    UltraStructLog<Pose2d> reefPosePublisher = new UltraStructLog<>("Vision/Reef Offset", Pose2d.struct);

    public enum LEDState {
        Aligned,
        Barge,
        Reef,
        Alliance,
    }
    
    @Override
    public void periodic() {
        LEDState ledState = LEDState.Alliance;
        Optional<Translation2d> bargePose = getBargePose();
        Optional<Translation2d> reefPose = getReefPose();

        if (bargePose.isPresent()) {
            bargePosePublisher.update(bargePose.get());
        }

        if (reefPose.isPresent()) {
            Translation2d e = reefPose.get();
            reefPosePublisher.update(new Pose2d(e.getX(), e.getY(), Rotation2d.kZero));
        }

        SmartDashboard.putNumber("Vision Aligned Timestamp", Math.abs(RobotController.getFPGATime() - isAlignedTimestamp) / 1e6);

        if (reefCamera.hasTarget() && tagIsInArray(reefCamera.getTargetID(), AutoConstants.reefTagIDs)) {
            ledState = LEDState.Reef;

            timeSinceReefTagSeen = Seconds.of(0);
            lastSeenReefTag = Optional.of(reefCamera.getInfo());
        } else {
            if (timeSinceReefTagSeen.gt(AutoConstants.lastPoseTimeout) && lastSeenReefTag.isPresent()) {
                lastSeenReefTag = Optional.empty();
            }
            timeSinceReefTagSeen = timeSinceReefTagSeen.plus(Milliseconds.of(20));
        }

        if (bargeCamera.hasTarget() && tagIsInArray(bargeCamera.getTargetID(), AutoConstants.bargeTagIDs)) {
            ledState = LEDState.Barge;

            timeSinceBargeTagSeen = Seconds.of(0);
            lastSeenBargeTag = Optional.of(bargeCamera.getInfo());
        } else {
            if (timeSinceBargeTagSeen.gt(AutoConstants.lastPoseTimeout) && lastSeenBargeTag.isPresent()) {
                lastSeenBargeTag = Optional.empty();
            }
            timeSinceBargeTagSeen = timeSinceBargeTagSeen.plus(Milliseconds.of(20));
        }

        if (isAligned()) {
            ledState = LEDState.Aligned;
        }

        switch (ledState) {
            case Aligned:
                this.m_LedSubsystem.applyPatternOnce(alignedColor);
                break;
            case Barge:
                this.m_LedSubsystem.applyPatternOnce(seeingColor);
                break;
            case Reef:
                this.m_LedSubsystem.applyPatternOnce(seeingColorReef);
                break;
            default:
            case Alliance:
                this.m_LedSubsystem.applyPatternOnce(this.m_LedSubsystem.allianceColorGetter());
        }
    }

    public Command resetLastPose() {
        return runOnce(() -> { lastSeenBargeTag = Optional.empty(); lastSeenReefTag = Optional.empty(); });
    }

    public Optional<Long> getBargeTagID() {
        long tid = bargeCamera.getTargetID();
        if (tid == -1) {
            return Optional.empty();
        }

        return Optional.of(tid);
    }

    public Optional<Long> getReefTagID() {
        long tid = reefCamera.getTargetID();
        if (tid == -1) {
            return Optional.empty();
        }

        return Optional.of(tid);
    }

    public boolean tagIsInArray(long tag, int[] arr) {
        for (int i = 0; i < arr.length; i++) {
            if (tag == arr[i])
                return true;
        }
        return false;
    }

    public boolean tagIsBarge() {
        if (lastSeenBargeTag.isEmpty()) return false;

        return tagIsInArray(lastSeenBargeTag.get().tid, AutoConstants.bargeTagIDs);
    }

    public boolean tagIsReef() {
        if (lastSeenReefTag.isEmpty()) return false;

        return tagIsInArray(lastSeenReefTag.get().tid, AutoConstants.reefTagIDs);
    }

    public Optional<Translation2d> getBargePose() {
        if (lastSeenBargeTag.isPresent() && tagIsBarge()) {
            double yAxisRotation = Math.toRadians(bargeCameraAngleYDegrees + lastSeenBargeTag.get().ty);
            double forwardDistance = (bargeTargetHeightMeters - bargeCameraHeightMeters) / Math.tan(yAxisRotation);

            double zAxisRotation = Math.toRadians(lastSeenBargeTag.get().tx);
            double sidewaysDistance = forwardDistance * Math.tan(zAxisRotation);

            return Optional.of(new Translation2d(Meters.of(forwardDistance), Meters.of(sidewaysDistance)));
        }
        return Optional.empty();
    }

    public Optional<Translation2d> getReefPose() {
        if (lastSeenReefTag.isPresent() && tagIsReef()) {
            double yAxisRotation = Math.toRadians(reefCameraAngleYDegrees + lastSeenReefTag.get().ty);
            double forwardDistance = (reefTagHeightMeters - reefCameraHeightMeters) / Math.tan(yAxisRotation);
            
            double zAxisRotation = Math.toRadians(reefCameraAngleZDegrees + lastSeenReefTag.get().tx);
            double sidewaysDistance = forwardDistance * Math.tan(zAxisRotation);

            return Optional.of(new Translation2d(Meters.of(forwardDistance), Meters.of(sidewaysDistance)));
        }
        return Optional.empty();
    }

    public boolean canSeeBargeTag() {
        return bargeCamera.hasTarget();
    }

    public boolean hasBargeTagInfo() {
        return lastSeenBargeTag.isPresent();
    }

    public boolean hasReefTagInfo() {
        return lastSeenReefTag.isPresent();
    }
}
