package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

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

import com.ctre.phoenix6.Timestamp;
import com.ctre.phoenix6.Timestamp.TimestampSource;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;

public final class LimelightCamera implements Camera {
    private String m_Name;
    private Transform3d m_Offset;
    private AprilTagFieldLayout m_Layout;

    private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    public LimelightCamera(String name, Transform3d offset, AprilTagFieldLayout layout) {
        m_Name = name;
        m_Offset = offset;
        m_Layout = layout;
    }

    public void update(Result result) {
        result.isNew = true;
        result.targetID = (int)limelightTable.getEntry("tid").getInteger(-1);

        if (result.targetID < 0) {
            result.cameraToTargetDistance = Meters.of(-1);
        } else {
            Angle angleToGoal = m_Offset.getRotation().getMeasureY().plus(Degrees.of(limelightTable.getEntry("ty").getDouble(0)));
            result.cameraToTargetDistance = m_Layout.getTagPose(result.targetID).get().getMeasureZ().minus(m_Offset.getMeasureZ()).div(Math.tan(angleToGoal.in(Radians)));
        }
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
    public void setReference(Pose2d pose) {}
}
