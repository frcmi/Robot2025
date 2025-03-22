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
import frc.robot.Constants.AutoConstants;
import frc.robot.vision.LimeLightAprilTag;
import frc.robot.vision.LimeLightAprilTag.TagInfo;
import frc.robot.subsystems.LEDSubsystem;

public final class TrigVisionSubsystem extends SubsystemBase {
    private LimeLightAprilTag bargeCamera;
    private LimeLightAprilTag reefCamera;
    private Optional<TagInfo> lastSeenTag = Optional.empty();
    private Time timeSinceTagSeen = Seconds.of(0);
    private LEDSubsystem m_LedSubsystem;

    private final double bargeCameraHeightMeters = Inches.of(10.5).in(Meters);   // Height of the camera off the ground in meters
    private final double bargeTargetHeightMeters = Inches.of(73).in(Meters);     // Height of the AprilTag on the field in meters
    private final double bargeCameraAngleYDegrees = AutoConstants.cameraAngle.in(Degrees);    // Angle at which the camera is mounted
    
    private final double reefTagHeightMeters = Inches.of(0.0).in(Meters);
    private final double reefCameraHeightMeters = Inches.of(0.0).in(Meters);
    private final double reefCameraAngleZDegrees = 0.0;
    private final double reefCameraAngleYDegrees = 0.0;

    public TrigVisionSubsystem(LEDSubsystem m_LedSubsystem) {
        bargeCamera = new LimeLightAprilTag("limelight-barge");
        reefCamera = new LimeLightAprilTag("limelight-reef");

        this.m_LedSubsystem = m_LedSubsystem;
    }

    LEDPattern seeingColor = LEDPattern.solid(new Color(255, 0, 255));
    LEDPattern alignedColor = LEDPattern.solid(new Color(0, 255, 0));
    // LEDPattern notSeeingColor = LEDPattern.rainbow(255, 128);

    public double isAlignedTimestamp = 0;

    public boolean isAligned() {
        return Math.abs(RobotController.getFPGATime() - isAlignedTimestamp) / 1e6 < 0.1;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Vision Aligned Timestamp", Math.abs(RobotController.getFPGATime() - isAlignedTimestamp) * 1e6);
        //jonas is racist NO I LOVE WOMEN
        if (bargeCamera.hasTarget()) {
            if (!isAligned()) {
                this.m_LedSubsystem.applyPatternOnce(seeingColor);
            } else {
                this.m_LedSubsystem.applyPatternOnce(alignedColor);
            }

            timeSinceTagSeen = Seconds.of(0);
            lastSeenTag = Optional.of(bargeCamera.getInfo());
        } else {
            if (timeSinceTagSeen.gt(AutoConstants.lastPoseTimeout) && lastSeenTag.isPresent()) {
                lastSeenTag = Optional.empty();
            }
            timeSinceTagSeen = timeSinceTagSeen.plus(Milliseconds.of(20));
            this.m_LedSubsystem.applyPatternOnce(this.m_LedSubsystem.allianceColorGetter());
        }
    }

    public Command resetLastPose() {
        return run(() -> { lastSeenTag = Optional.empty(); });
    }

    public Optional<Long> getTagID() {
        long tid = bargeCamera.getTargetID();
        if (tid == -1) {
            return Optional.empty();
        }

        return Optional.of(tid);
    }

    private boolean tagIsInArray(int[] arr) {
        if (lastSeenTag.isEmpty()) return false;
        for (int i = 0; i < arr.length; i++) {
            if (lastSeenTag.get().tid == arr[i])
                return true;
        }
        return false;
    }

    public boolean tagIsBarge() {
        return tagIsInArray(AutoConstants.bargeTagIDs);
    }

    public boolean tagIsReef() {
        return tagIsInArray(AutoConstants.reefTagIDs);
    }

    public Optional<Translation2d> getBargePose() {
        if (lastSeenTag.isPresent() && tagIsBarge()) {
            double xAxisRotation = Math.toRadians(bargeCameraAngleYDegrees + lastSeenTag.get().ty);
            double forwardDistance = (bargeTargetHeightMeters - bargeCameraHeightMeters) / Math.tan(xAxisRotation);
            
            double zAxisRotation = Math.toRadians(lastSeenTag.get().tx);
            double sidewaysDistance = forwardDistance * Math.tan(zAxisRotation);

            return Optional.of(new Translation2d(Meters.of(sidewaysDistance), Meters.of(forwardDistance)));
        }
        return Optional.empty();
    }

    public Optional<Translation2d> getReefPose() {
        if (lastSeenTag.isPresent() && tagIsReef()) {
            double xAxisRotation = Math.toRadians(reefCameraAngleYDegrees + lastSeenTag.get().ty);
            double forwardDistance = (reefTagHeightMeters - reefCameraHeightMeters) / Math.tan(xAxisRotation);
            
            double zAxisRotation = Math.toRadians(lastSeenTag.get().tx);
            double sidewaysDistance = forwardDistance * Math.tan(zAxisRotation);

            return Optional.of(new Translation2d(Meters.of(sidewaysDistance), Meters.of(forwardDistance)));
        }
        return Optional.empty();
    }

    public boolean canSeeTag() {
        return bargeCamera.hasTarget();
    }
    public boolean hasTagInfo() {
        return lastSeenTag.isPresent();
    }
}
