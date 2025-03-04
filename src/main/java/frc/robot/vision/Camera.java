package frc.robot.vision;

import com.ctre.phoenix6.Timestamp;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public interface Camera {
    public static class Result {
        public Distance cameraToTargetDistance;
        public Rotation3d cameraToTargetRotation;
        public int targetID;
                
        public boolean isNew;
    }

    public void update(Result result);
    
    public void setReference(Pose2d pose);

    public String getName();
    public Transform3d getOffset();
}
