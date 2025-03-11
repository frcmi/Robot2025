package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Microseconds;
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
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.vision.LimeLightAprilTag;
import frc.robot.subsystems.LEDSubsystem;

public final class TrigVisionSubsystem extends SubsystemBase {
    private LimeLightAprilTag tag;
    private StructPublisher<Pose2d> tagRelativePosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Vision/Position to Barge", Pose2d.struct).publish();
    private Optional<Pose2d> lastRecordedPose = Optional.empty();
    private Time timeSinceTagSeen = Seconds.of(0);
    private LEDSubsystem m_LedSubsystem;;

    public TrigVisionSubsystem(LEDSubsystem m_LedSubsystem) {
        tag = new LimeLightAprilTag();
        this.m_LedSubsystem = m_LedSubsystem;
    }

    LEDPattern seeingColor = LEDPattern.solid(new Color(255, 0, 255));
    LEDPattern alignedColor = LEDPattern.solid(new Color(0, 255, 0));
    LEDPattern notSeeingColor = LEDPattern.rainbow(255, 128);

    public double isAlignedTimestamp = 0;

    public boolean isAligned() {
        return Math.abs(RobotController.getFPGATime() - isAlignedTimestamp) / 1e6 < 0.1;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Vision Aligned Timestamp", Math.abs(RobotController.getFPGATime() - isAlignedTimestamp) * 1e6);
        if (tag.hasTarget()) {
            if (!isAligned()) {
                this.m_LedSubsystem.applyPatternOnce(seeingColor);
            } else {
                this.m_LedSubsystem.applyPatternOnce(alignedColor);
            }

            timeSinceTagSeen = Seconds.of(0);
            lastRecordedPose = Optional.of(new Pose2d(Meters.of(tag.getVerticalDistanceMeters()), Meters.of(tag.getLateralDistanceMeters()), new Rotation2d()));
            tagRelativePosePublisher.set(lastRecordedPose.get());
        } else {
            if (timeSinceTagSeen.gt(AutoConstants.lastPoseTimeout) && lastRecordedPose.isPresent()) {
                lastRecordedPose = Optional.empty();
            }
            timeSinceTagSeen = timeSinceTagSeen.plus(Milliseconds.of(20));
            this.m_LedSubsystem.applyPatternOnce(notSeeingColor);
        }
    }

    public Command resetLastPose() {
        return run(() -> { lastRecordedPose = Optional.empty(); });
    }

    public Optional<Long> getTagID() {
        long tid = tag.getTargetID();
        if (tid == -1) {
            return Optional.empty();
        }

        return Optional.of(tid);
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
