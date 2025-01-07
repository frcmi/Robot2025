package frc.robot.subsystems;

import java.io.IOException;
import java.util.HashSet;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Robot;
import frc.robot.vision.Camera;
import frc.robot.vision.PhotonlibCamera;
import frc.robot.vision.Camera.Result;
import frc.robot.vision.Camera.Simulator;
import frc.robot.vision.Camera.Specification;

public final class VisionSubsystem implements Subsystem {
    public static final double kMaxAmbiguity = 0.7;
    public static final double kMaxDistance = Units.feetToMeters(10);

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

    private SwerveSubsystem m_Swerve;
    private CameraData[] m_Cameras;

    private HashSet<Integer> m_ViableResults;
    private int m_Frame;

    public static VisionSubsystem configure(SwerveSubsystem swerve) {
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

    public VisionSubsystem(SwerveSubsystem swerve, CameraDescription[] cameras, AprilTagFieldLayout layout) {
        m_Swerve = swerve;
        m_ViableResults = new HashSet<>();
        m_Frame = 0;

        m_Cameras = new CameraData[cameras.length];
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

        if (result.maxDistance > kMaxDistance) {
            return false;
        }

        if (result.maxAmbiguity > kMaxAmbiguity) {
            return false;
        }

        return true;
    }

    public Result getResult(int camera) {
        return m_Cameras[camera].result;
    }

    @Override
    public void periodic() {
        m_ViableResults.clear();

        for (int i = 0; i < m_Cameras.length; i++) {
            var result = m_Cameras[i].result;
            m_Cameras[i].camera.update(result);

            if (isResultViable(result)) {
                m_ViableResults.add(i);
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        var pose = m_Swerve.getState().Pose;

        for (var camera : m_Cameras) {
            camera.sim.update(pose, m_Frame);
        }

        m_Frame++;
    }
}
