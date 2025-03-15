package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import java.io.IOException;
import java.util.HashSet;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Robot;
import frc.robot.vision.Camera;
import frc.robot.vision.LimelightCamera;
import frc.robot.vision.PhotonlibCamera;
import frc.robot.vision.Camera.Result;
import frc.robot.vision.Camera.Simulator;
import frc.robot.vision.Camera.Specification;

public final class GlobalVisionSubsystem implements Subsystem {
    public static final double kMaxAmbiguity = 0.7;
    public static final double kMaxDistance = Units.feetToMeters(10);

    public static final Translation3d kLimelightTranslation = new Translation3d(0, 0, 0);
    public static final Rotation3d kLimelightRotation = new Rotation3d(
        Radians.of(0),
        Radians.of(0),
        Radians.of(0));

    public static final Transform3d kLimelightTransform = new Transform3d(kLimelightTranslation, kLimelightRotation);
    public static final CameraDescription[] kCameras = new CameraDescription[] {
        new CameraDescription("limelight", CameraType.LIMELIGHT, kLimelightTransform, null)
    };

    public static enum CameraType {
        PHOTONVISION,
        LIMELIGHT
    }

    public static class CameraDescription {
        String name;
        CameraType type;
        Transform3d offset;
        Specification spec;

        public CameraDescription(String name, CameraType type, Transform3d offset, Specification spec) {
            this.name = name;
            this.type = type;
            this.offset = offset;
            this.spec = spec;
        }
    };

    private static class CameraData {
        CameraType type;
        Camera camera;
        Result result;
        Simulator sim;
    }

    private CommandSwerveDrivetrain m_Swerve;
    private CameraData[] m_Cameras;

    private HashSet<Integer> m_ViableResults;
    private int m_Frame;
    private AprilTagFieldLayout m_Layout;

    public static GlobalVisionSubsystem configure(CommandSwerveDrivetrain swerve) {
        AprilTagFieldLayout layout;
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException exc) {
            System.err.println("no such april tag field");
            layout = null;
        }

        return new GlobalVisionSubsystem(swerve, layout);
    }

    private static Camera createCamera(CameraDescription desc, AprilTagFieldLayout layout) {
        switch (desc.type) {
            case PHOTONVISION:
                return new PhotonlibCamera(desc.name, desc.offset, layout);
            case LIMELIGHT:
                return new LimelightCamera(desc.name, desc.offset);
            default:
                return null;
        }
    }

    public GlobalVisionSubsystem(CommandSwerveDrivetrain swerve, AprilTagFieldLayout layout) {
        m_Swerve = swerve;
        m_ViableResults = new HashSet<>();
        m_Frame = 0;

        m_Cameras = new CameraData[kCameras.length];
        for (int i = 0; i < kCameras.length; i++) {
            var desc = kCameras[i];
            var data = new CameraData();

            data.type = desc.type;
            data.camera = createCamera(desc, layout);
            data.result = new Result();
            data.sim = null;

            if (Robot.isSimulation() && desc.spec != null) {
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

    public AprilTagFieldLayout getLayout() {
        return m_Layout;
    }

    public int getCameraCount() {
        return m_Cameras.length;
    }

    public Camera getCamera(int camera) {
        return m_Cameras[camera].camera;
    }

    public CameraType getCameraType(int camera) {
        return m_Cameras[camera].type;
    }

    public Result getResult(int camera) {
        return m_Cameras[camera].result;
    }

    /**
     * Finds the first camera of the given type. Returns -1 on failure.
     * 
     * @param type Camera type to search for.
     * @return The index of the camera.
     */
    public int getCameraByType(CameraType type) {
        for (int i = 0; i < m_Cameras.length; i++) {
            var data = m_Cameras[i];
            if (data.type == type) {
                return i;
            }
        }

        return -1;
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
            if (camera.sim == null) {
                continue;
            }

            camera.sim.update(pose, m_Frame);
        }

        m_Frame++;
    }
}