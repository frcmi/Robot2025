package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import frc.lib.LimelightHelpers;

public class LimelightCamera implements Camera {
    private String m_Name;
    private Transform3d m_Offset;

    public LimelightCamera(String name, Transform3d offset) {
        m_Name = name;
        m_Offset = offset;

        var translation = m_Offset.getTranslation();
        var rotation = m_Offset.getRotation();

        double radiansToDegrees = 180 / Math.PI;
        double roll = rotation.getX() * radiansToDegrees;
        double pitch = rotation.getY() * radiansToDegrees;
        double yaw = rotation.getZ() * radiansToDegrees;

        LimelightHelpers.setCameraPose_RobotSpace(m_Name,
                translation.getX(), translation.getY(), translation.getZ(),
                roll, pitch, yaw);
    }

    @Override
    public void update(Result result) {
        var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_Name);
        result.isNew = false;

        if (estimate.tagCount <= 0) {
            return;
        }

        for (int i = 0; i < estimate.tagCount; i++) {
            int id = estimate.rawFiducials[i].id;
            if (id < 1 || id > 16) {
                return;
            }
        }

        result.pose = estimate.pose;
        result.timestamp = estimate.timestampSeconds;
        result.isNew = true;

        result.maxAmbiguity = Double.MIN_VALUE;
        result.maxDistance = Double.MIN_VALUE;
        result.minDistance = Double.MAX_VALUE;

        result.tags = new Tag[estimate.tagCount];
        for (int i = 0; i < estimate.tagCount; i++) {
            var fiducial = estimate.rawFiducials[i];

            result.maxAmbiguity = Math.max(result.maxAmbiguity, fiducial.ambiguity);
            result.maxDistance = Math.max(result.maxDistance, fiducial.distToCamera);
            result.minDistance = Math.min(result.minDistance, fiducial.distToCamera);

            var tag = new Tag();
            tag.ID = fiducial.id;
            tag.cameraDistance = fiducial.distToCamera;
            result.tags[i] = tag;
        }
    }

    @Override
    public void setReference(Pose2d pose) {
        var yaw = pose.getRotation();
        LimelightHelpers.SetRobotOrientation(m_Name, yaw.getDegrees(), 0, 0, 0, 0, 0);
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
        throw new UnsupportedOperationException("The Limelight camera does not have a simulator");
    }
}