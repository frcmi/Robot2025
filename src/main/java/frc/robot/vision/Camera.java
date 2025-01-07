package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public interface Camera {
    public static interface Simulator {
        public void update(Pose2d pose, int frame);
        public void reset(Pose2d pose);
    }

    public static class Result {
        public Pose2d pose;
        public Transform3d cameraToTarget;
        public int targetID;
        public double maxAmbiguity, maxDistance, minDistance;
        
        public boolean isNew;
        public double timestamp;
    }

    public static class Specification {
        public Specification(int width, int height, Rotation2d fov, double meanError, double stdDevError, double meanLatency, double stdDevLatency, double fps) {
            this.width = width;
            this.height = height;
            this.fov = fov;
            this.meanError = meanError;
            this.stdDevError = stdDevError;
            this.meanLatency = meanLatency;
            this.stdDevLatency = stdDevLatency;
            this.fps = fps;
        }

        public int width, height;
        public Rotation2d fov;
        public double meanError, stdDevError;
        public double meanLatency, stdDevLatency;
        public double fps;
    };

    public void update(Result result);
    public void setReference(Pose2d pose);

    public String getName();
    public Transform3d getOffset();

    public Simulator createSimulator(Specification specification);
}
