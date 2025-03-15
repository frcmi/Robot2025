package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.HashSet;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.GlobalVisionSubsystem.CameraType;
import frc.robot.vision.Camera.Tag;

public final class TrigVisionSubsystem extends SubsystemBase {
    private StructPublisher<Translation3d> tagRelativePosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Vision/Position to Barge", Translation3d.struct).publish();
    private Optional<Translation3d> lastRecordedOffset = Optional.empty();
    private Time timeSinceTagSeen = Seconds.of(0);
    private LEDSubsystem leds;
    private GlobalVisionSubsystem globalVision;
    private int cameraIndex;

    private static final HashSet<Integer> kValidIDs;

    static {
        kValidIDs = new HashSet<>();

        var validIDs = new int[] {
                14,
                15,
                4,
                5
        };

        for (int i = 0; i < validIDs.length; i++) {
            kValidIDs.add(validIDs[i]);
        }
    }

    public TrigVisionSubsystem(LEDSubsystem leds, GlobalVisionSubsystem globalVision) {
        this.leds = leds;
        this.globalVision = globalVision;

        // just use the limelight
        cameraIndex = globalVision.getCameraByType(CameraType.LIMELIGHT);
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
        SmartDashboard.putNumber("Vision Aligned Timestamp",
                Math.abs(RobotController.getFPGATime() - isAlignedTimestamp) * 1e6);

        var tag = getBestTag();
        if (tag != null) {
            if (!isAligned()) {
                leds.applyPatternOnce(seeingColor);
            } else {
                leds.applyPatternOnce(alignedColor);
            }

            var camera = globalVision.getCamera(cameraIndex);
            var cameraOffset = camera.getOffset();

            var tagRotation = cameraOffset.getRotation().plus(tag.rotationOffset);
            var pitch = tagRotation.getMeasureY();
            var yaw = tagRotation.getMeasureY();
            var horizontalDistance = tag.cameraDistance.times(Math.cos(pitch.in(Radians)));
            var verticalDistance = tag.cameraDistance.times(Math.sin(pitch.in(Radians)));
            
            var directDistance = horizontalDistance.times(Math.cos(yaw.in(Radians)));
            var perpendicularDistance = horizontalDistance.times(Math.sin(yaw.in(Radians)));
            var cameraToTag = new Translation3d(directDistance, perpendicularDistance, verticalDistance);

            var robotToCamera = cameraOffset.getTranslation();
            var robotToTag = robotToCamera.plus(cameraToTag);

            timeSinceTagSeen = Seconds.of(0);
            lastRecordedOffset = Optional.of(robotToTag);
            tagRelativePosePublisher.set(robotToTag);
        } else {
            if (timeSinceTagSeen.gt(AutoConstants.lastPoseTimeout)) {
                lastRecordedOffset = Optional.empty();
            }
            
            timeSinceTagSeen = timeSinceTagSeen.plus(Milliseconds.of(20));
            leds.applyPatternOnce(leds.allianceColorGetter());
        }
    }

    public Command reset() {
        return run(() -> {
            lastRecordedOffset = Optional.empty();
        });
    }

    public Tag getBestTag() {
        if (cameraIndex < 0) {
            return null;
        }

        var result = globalVision.getResult(cameraIndex);
        if (!result.isNew) {
            return null;
        }

        int bestTag = -1;
        Distance minDistance = null;

        for (int i = 0; i < result.tags.length; i++) {
            var tag = result.tags[i];
            if (!kValidIDs.contains(tag.ID)) {
                continue;
            }

            if (minDistance == null || tag.cameraDistance.lt(minDistance)) {
                bestTag = i;
                minDistance = tag.cameraDistance;
            }
        }

        return bestTag < 0 ? null : result.tags[bestTag];
    }

    /**
     * Returns the last calculated position relative to the barge.
     */
    public Optional<Translation3d> getRobotToTag() {
        return lastRecordedOffset;
    }
}
