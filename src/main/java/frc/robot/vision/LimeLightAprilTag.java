package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.AutoConstants;

public class LimeLightAprilTag {
    public class TagInfo {
        public int tid;
        public double tx;
        public double ty;

        public TagInfo(int id, double tx, double ty) {
            this.tid = id;
            this.tx = tx;
            this.ty = ty;
        }
    }
    // Get the LimeLight NetworkTable. Adjust the table name if necessary.
    private final NetworkTable limeTable;
    // Camera and target configuration (modify these to match your setup)
    public LimeLightAprilTag(String limelightName) {
        limeTable = NetworkTableInstance.getDefault().getTable(limelightName);
    }
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

    public TagInfo getInfo() {
        return new TagInfo((int)getTargetID(), getHorizontalOffset(), getVerticalOffset());
    }
}