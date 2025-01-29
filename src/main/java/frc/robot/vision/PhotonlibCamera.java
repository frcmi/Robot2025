package frc.robot.vision;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public final class PhotonlibCamera implements Camera {
    private static final class PhotonlibSimulator implements Camera.Simulator {
        private static VisionSystemSim s_System;
        private static int s_Frame;

        static {
            s_System = new VisionSystemSim("main");
            s_Frame = -1;
        }

        public PhotonlibSimulator(PhotonCameraSim sim, Transform3d transform) {
            s_System.addCamera(sim, transform);
        }

        @Override
        public void update(Pose2d pose, int frame) {
            if (s_Frame == frame) {
                return;
            }

            s_System.update(pose);
            s_Frame = frame;
        }

        @Override
        public void reset(Pose2d pose) {
            s_System.resetRobotPose(pose);
        }
        
    }

    private String m_Name;
    private Transform3d m_Offset;

    private PhotonCamera m_Camera;
    private PhotonPoseEstimator m_Estimator;

    public PhotonlibCamera(String name, Transform3d offset, AprilTagFieldLayout layout) {
        m_Name = name;
        m_Offset = offset;

        m_Camera = new PhotonCamera(name);
        m_Estimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, offset);
    }

    private boolean isResultValid(PhotonPipelineResult result) {
        if (!result.hasTargets()) {
            return false;
        }

        for (var target : result.getTargets()) {
            int id = target.getFiducialId();
            if (id < 1 || id > 8) {
                return false;
            }
        }

        return true;
    }

    private void update(Result result, PhotonPipelineResult cameraResult) {
        if (!isResultValid(cameraResult)) {
            return;
        }

        PhotonTrackedTarget target = cameraResult.getBestTarget();

        result.isNew = true;
        result.targetID = target.getFiducialId();

        var transform = target.getBestCameraToTarget();
        result.cameraToTargetDistance = Meters.of(transform.getTranslation().getNorm());
        result.cameraToTargetRotation = transform.getRotation();
    }

    @Override
    public void update(Result result) {
        result.isNew = false;

        var results = m_Camera.getAllUnreadResults();
        for (var currentResult : results) {
            update(result, currentResult);
        }
    }

    @Override
    public void setReference(Pose2d pose) {
        m_Estimator.setReferencePose(pose);
    }

    @Override
    public String getName() {
        return m_Name;
    }

    @Override
    public Transform3d getOffset() {
        return m_Offset;
    }

    @Override
    public Simulator createSimulator(Specification specification) {
        var properties = new SimCameraProperties();
        properties.setCalibration(specification.width, specification.height, specification.fov);
        properties.setFPS(specification.fps);
        properties.setAvgLatencyMs(specification.meanLatency);
        properties.setLatencyStdDevMs(specification.stdDevLatency);
        properties.setCalibError(specification.meanError, specification.stdDevError);
        
        var sim = new PhotonCameraSim(m_Camera, properties);
        return new PhotonlibSimulator(sim, m_Offset);
    }

}
