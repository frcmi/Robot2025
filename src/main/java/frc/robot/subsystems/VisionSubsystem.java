package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.io.IOException;
import java.util.HashSet;

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
import frc.robot.vision.LimelightCamera;
import frc.robot.vision.PhotonlibCamera;
import frc.robot.vision.Camera.Result;

public final class VisionSubsystem extends SubsystemBase {
    AprilTagFieldLayout fieldLayout;

    public static enum CameraType {
        PHOTONVISION,
        LIMELIGHT
    }

    public static class CameraDescription {
        String name;
        CameraType type;
        Transform3d offset;

        public CameraDescription(String name, CameraType type, Transform3d offset) {
            this.name = name;
            this.type = type;
            this.offset = offset;
        }
    };

    private Camera m_Camera;
    private Result result;
    private LimeLightAprilTag tag;

    public VisionSubsystem(CameraDescription camera, String fieldName) {
        // try {
        //     fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        // } catch (Exception e) {
        //     System.out.println("No april tag map called " + fieldName);
        //     System.out.println(e);
        // }

        // var desc = camera;
        // result = new Result(); 

        // switch (desc.type) {
        //     case PHOTONVISION:
        //         m_Camera = new PhotonlibCamera(desc.name, desc.offset, fieldLayout);
        //     default:
        //         m_Camera = new LimelightCamera(desc.name, desc.offset, fieldLayout);
        // }
        tag = new LimeLightAprilTag();
    }

    DoublePublisher cameraPublisher = NetworkTableInstance.getDefault()
        .getDoubleTopic("Test/Camera Distance").publish();

    @Override
    public void periodic() {
        // m_Camera.update(result);
        if (tag.hasTarget()) {
            cameraPublisher.set(Meters.of(tag.getDistanceMeters()).in(Inches));
            
        }
        // result.isNew = false;

        // var result = m_Camera.result;
        // m_Camera.camera.update(result);

        // if (result.isNew) {
        //     Transform3d cameraOffset = m_Camera.camera.getOffset();
        //     double pitchToTarget = 
        //         cameraOffset.getRotation().getMeasureY()
        //         .plus(
        //             result.cameraToTargetRotation.getMeasureY()).in(Radians);

        //     double planarDistanceToTarget = result.cameraToTargetDistance.in(Meters) * Math.cos(pitchToTarget);

        //     Rotation3d botRotation = 
        //         new Rotation3d(
        //             Rotations.of(0), 
        //             Rotations.of(0), 
        //             Rotations.of(0)
        //         );

        //     Rotation3d botToTargetRotation =
        //         botRotation
        //         .rotateBy(cameraOffset.getRotation())
        //         .rotateBy(result.cameraToTargetRotation);

        //     Translation3d botToTagTranslation =
        //         new Translation3d(
        //             planarDistanceToTarget, new Rotation3d()
        //         ).rotateBy(
        //             botToTargetRotation.times(-1)
        //         );

        //     Translation3d botTranslation = 
        //         fieldLayout.getTagPose(result.targetID).get().getTranslation()
        //         .plus(botToTagTranslation);
            
        //     result.isNew = false;
        // }
    }
}
