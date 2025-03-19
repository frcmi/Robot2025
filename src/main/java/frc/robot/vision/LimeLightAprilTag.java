package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.AutoConstants;

public class LimeLightAprilTag {
    // Get the LimeLight NetworkTable. Adjust the table name if necessary.
    private final NetworkTable limeTable = NetworkTableInstance.getDefault().getTable("limelight");
    // Camera and target configuration (modify these to match your setup)
    private final double cameraHeightMeters = Inches.of(10.5).in(Meters);   // Height of the camera off the ground in meters
    private final double targetHeightMeters = Inches.of(73).in(Meters);     // Height of the AprilTag on the field in meters
    private final double cameraAngleDegrees = AutoConstants.cameraAngle.in(Degrees);    // Angle at which the camera is mounted
    // 3, 4.75, 10.5

    /**
Checks if a valid target is detected.
@return true if target is detected; false otherwise.
     */
    public boolean hasTarget() {
        return limeTable.getEntry("tv").getDouble(0) == 1.0;
    }
    /**
Gets the horizontal offset (tx) in degrees.
@return Horizontal offset from the crosshair.
     */
    public double getHorizontalOffset() {
        return limeTable.getEntry("tx").getDouble(0);
    }
    /**
Gets the vertical offset (ty) in degrees.
@return Vertical offset from the crosshair.
     */
    public double getVerticalOffset() {
        return limeTable.getEntry("ty").getDouble(0);
    }
    /**
Gets the target area (ta).
@return The area of the detected target.
     */
    public double getTargetArea() {
        return limeTable.getEntry("ta").getDouble(0);
    }
    /**
Gets the target id (tod).
@return The ID of the detected target.
     */
    public long getTargetID() {
        return limeTable.getEntry("tid").getInteger(-1);
    }
    /**
Computes an approximate distance to the target using the vertical offset.
Uses the formula:
   distance = (targetHeight - cameraHeight) / tan(cameraAngle + ty)
Make sure to convert angles to radians.
     *
@return Estimated distance in meters.
     */
    public double getLateralDistanceMeters() {
        double ty = getVerticalOffset();
        // Combine the mounting angle with the offset
        double angleToTargetRadians = Math.toRadians(cameraAngleDegrees + ty);
        return (targetHeightMeters - cameraHeightMeters) / Math.tan(angleToTargetRadians);
    }
    public double getVerticalDistanceMeters() {
        double tx = getHorizontalOffset();
        double verticalDist = getLateralDistanceMeters();

        return verticalDist * Math.tan(Degrees.of(tx).in(Radians));
    }
    /**
Example method to update and print the current LimeLight data.
     */
    public void update() {
        if (hasTarget()) {
            double tx = getHorizontalOffset();
            double ty = getVerticalOffset();
            double distance = getVerticalDistanceMeters();
            System.out.println("Target Detected!");
            System.out.println("Horizontal Offset (tx): " + tx + " degrees");
            System.out.println("Vertical Offset (ty): " + ty + " degrees");
            System.out.println("Estimated Distance: " + distance + " meters");
        } else {
            System.out.println("No target detected.");
        }
    }
}