package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import java.util.Optional;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.LimeLightAprilTag;

public final class TrigVisionSubsystem extends SubsystemBase {
    private LimeLightAprilTag tag;
    private DoublePublisher cameraPublisher = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/Barge Distance").publish();

    public TrigVisionSubsystem() {
        tag = new LimeLightAprilTag();
    }

    @Override
    public void periodic() {
        if (tag.hasTarget()) {
            cameraPublisher.set(Meters.of(tag.getDistanceMeters()).in(Inches));
        }
    }

    public Optional<Double> distanceFromBargeMeters() {
        // TODO: VERY IMPORTANT- MAKE THIS THE BARGE TAG ID
        // In testing, this should be 1 because that is the ID we are testing with,
        // however on field keeping this as 1 WILL MAKE IT NOT WORK
        if (tag.hasTarget() && tag.getTargetID() == 1) {
            return Optional.of(tag.getDistanceMeters());
        }
        return Optional.empty();
    }

    public double distanceFromBargeMeters(double defaultValue) {
        Optional<Double> dist = distanceFromBargeMeters();
        if (!dist.isEmpty()) {
            return dist.get().doubleValue();
        } else {
            return defaultValue;
        }
    }
}
