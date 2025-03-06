package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import java.util.Optional;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.LimeLightAprilTag;

public final class TrigVisionSubsystem extends SubsystemBase {
    private LimeLightAprilTag tag;
    private DoublePublisher cameraPublisher = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/Camera Distance").publish();

    public TrigVisionSubsystem() {
        tag = new LimeLightAprilTag();
    }

    @Override
    public void periodic() {
        if (tag.hasTarget()) {
            cameraPublisher.set(tag.getDistanceMeters());
            
        }
    }

    public Optional<Angle> getHorizontalRotation() {
        if (tag.hasTarget()) {
            return Optional.of(Degrees.of(tag.getHorizontalOffset()));
        } else {
            return Optional.empty();
        }
    }

    public Optional<Distance> distanceToBarge() {
        if (tag.hasTarget()) {
            return Optional.of(Meters.of(tag.getDistanceMeters()));
        } else {
            return Optional.empty();
        }
    }

    public double distanceToBarge(double defaultValue) {
        Optional<Distance> dist = distanceToBarge();

        if (dist.isPresent()) {
            return dist.get().in(Meters);
        } else {
            return defaultValue;
        }
    }

    public boolean canSeeTag() {
        return tag.hasTarget();
    }
}
