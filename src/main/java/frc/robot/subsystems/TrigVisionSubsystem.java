package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.vision.LimeLightAprilTag;

public final class TrigVisionSubsystem extends SubsystemBase {
    private LimeLightAprilTag tag;
    private StructPublisher<Pose2d> tagRelativePosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Vision/Position to Barge", Pose2d.struct).publish();
    private Optional<Pose2d> lastRecordedPose = Optional.empty();
    private Time timeSinceTagSeen = Seconds.of(0);

    public TrigVisionSubsystem() {
        tag = new LimeLightAprilTag();
    }

    @Override
    public void periodic() {
        if (tag.hasTarget()) {
            timeSinceTagSeen = Seconds.of(0);
            lastRecordedPose = Optional.of(new Pose2d(Meters.of(tag.getVerticalDistanceMeters()), Meters.of(tag.getLateralDistanceMeters()), new Rotation2d()));
            tagRelativePosePublisher.set(lastRecordedPose.get());
        } else {
            if (timeSinceTagSeen.gt(AutoConstants.lastPoseTimeout) && lastRecordedPose.isPresent()) {
                lastRecordedPose = Optional.empty();
            }
            timeSinceTagSeen = timeSinceTagSeen.plus(Milliseconds.of(20));
        }
    }

    public Command resetLastPose() {
        return run(() -> { lastRecordedPose = Optional.empty(); });
    }

    public Optional<Angle> getHorizontalRotation() {
        if (tag.hasTarget()) {
            return Optional.of(Degrees.of(tag.getHorizontalOffset()));
        } else {
            return Optional.empty();
        }
    }

    public Optional<Distance> getLateralDistanceToBarge() {
        if (lastRecordedPose.isPresent()) {
            return Optional.of(lastRecordedPose.get().getMeasureY());
        } else {
            return Optional.empty();
        }
    }

    public Optional<Distance> getVerticalDistanceToBarge() {
        if (lastRecordedPose.isPresent()) {
            return Optional.of(lastRecordedPose.get().getMeasureX());
        } else {
            return Optional.empty();
        }
    }

    public boolean canSeeTag() {
        return tag.hasTarget();
    }
    public boolean hasLastPosition() {
        return lastRecordedPose.isPresent();
    }
}
