package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.io.IOException;
import java.util.HashSet;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.vision.Camera;
import frc.robot.vision.LimeLightAprilTag;
import frc.robot.vision.LimelightCamera;
import frc.robot.vision.PhotonlibCamera;
import frc.robot.vision.Camera.Result;

public final class TrigVisionSubsystem extends SubsystemBase {
    private LimeLightAprilTag tag;
    private DoublePublisher cameraPublisher = NetworkTableInstance.getDefault()
        .getDoubleTopic("Test/Camera Distance").publish();

    public TrigVisionSubsystem() {
        tag = new LimeLightAprilTag();
    }

    @Override
    public void periodic() {
        if (tag.hasTarget()) {
            cameraPublisher.set(Meters.of(tag.getDistanceMeters()).in(Inches));
            
        }
    }

    public Optional<Double> distanceToBarge() {
        if (tag.hasTarget()) {
            return Optional.of(tag.getDistanceMeters());
        } else {
            return Optional.empty();
        }
    }

    public double distanceToBarge(double defaultValue) {
        Optional<Double> dist = distanceToBarge();

        if (dist.isPresent()) {
            return dist.get();
        } else {
            return defaultValue;
        }
    }
}
