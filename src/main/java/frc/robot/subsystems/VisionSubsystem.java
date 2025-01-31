package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Robot;
import frc.robot.vision.Camera;
import frc.robot.vision.PhotonlibCamera;
import frc.robot.vision.Camera.Result;
import frc.robot.vision.Camera.Simulator;
import frc.robot.vision.Camera.Specification;

public final class VisionSubsystem implements Subsystem {
    AprilTagFieldLayout fieldLayout;

    public static enum CameraType {
        PHOTONVISION
    }

    public static class CameraDescription {
        String name;
        CameraType type;
        Transform3d offset;
        Specification spec;
    };

    private static class CameraData {
        Camera camera;
        Result result;
        Simulator sim;
    }

    private CommandSwerveDrivetrain m_Swerve;
    private CameraData m_Camera;

    private int m_Frame;

    public static VisionSubsystem configure(CommandSwerveDrivetrain swerve) {
        var cameras = new CameraDescription[] { /* cameras */ };

        AprilTagFieldLayout layout;
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException exc) {
            System.err.println("no such april tag field");
            layout = null;
        }

        return new VisionSubsystem(swerve, cameras, layout);
    }

    private static Camera createCamera(CameraDescription desc, AprilTagFieldLayout layout) {
        switch (desc.type) {
            case PHOTONVISION:
                return new PhotonlibCamera(desc.name, desc.offset, layout);
            default:
                return null;
        }
    }

    public VisionSubsystem(CommandSwerveDrivetrain swerve, CameraDescription[] cameras, AprilTagFieldLayout layout) {
        m_Swerve = swerve;
        m_Frame = 0;
        fieldLayout = layout;

        m_Camera = new CameraData();
        for (int i = 0; i < cameras.length; i++) {
            var desc = cameras[i];
            var data = new CameraData();

            data.camera = createCamera(desc, layout);
            data.result = new Result();

            if (Robot.isSimulation()) {
                data.sim = data.camera.createSimulator(desc.spec);
            }
        }
    }

    private static boolean isResultViable(Result result) {
        if (!result.isNew) {
            return false;
        }

        return true;
    }

    public Result getResult() {
        return m_Camera.result;
    }

    @Override
    public void periodic() {
        var result = m_Camera.result;
        m_Camera.camera.update(result);

        if (isResultViable(result)) {
            Transform3d cameraOffset = m_Camera.camera.getOffset();
            double pitchToTarget = 
                cameraOffset.getRotation().getMeasureY()
                .plus(
                    result.cameraToTargetRotation.getMeasureY()).in(Radians);

            double planarDistanceToTarget = result.cameraToTargetDistance.in(Meters) * Math.cos(pitchToTarget);

            Rotation3d botRotation = 
                new Rotation3d(
                    Rotations.of(0), 
                    Rotations.of(0), 
                    m_Swerve.getState().Pose.getRotation().getMeasure()
                );

            Rotation3d botToTargetRotation =
                botRotation
                .rotateBy(cameraOffset.getRotation())
                .rotateBy(result.cameraToTargetRotation);

            Translation3d botToTagTranslation =
                new Translation3d(
                    planarDistanceToTarget, new Rotation3d()
                ).rotateBy(
                    botToTargetRotation.times(-1)
                );

            Translation3d botTranslation = 
                fieldLayout.getTagPose(result.targetID).get().getTranslation()
                .plus(botToTagTranslation);

            m_Swerve.addVisionMeasurement(
                new Pose2d(
                    botTranslation.getMeasureX(), 
                    botTranslation.getMeasureY(), 
                    new Rotation2d(
                        botToTargetRotation.getMeasureZ()
                    )
                ), 
                result.timestamp
            );
            
            result.isNew = false;
        }
    }

    @Override
    public void simulationPeriodic() {
        var pose = m_Swerve.getState().Pose;

        m_Camera.sim.update(pose, m_Frame);

        m_Frame++;
    }
}
