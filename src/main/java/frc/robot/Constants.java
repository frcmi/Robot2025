package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.*;

public final class Constants {
    public static class TelemetryConstants {
        // DON'T ENABLE UNLESS ABSOLUTELY NEEDED
        // this will fully disable logging even when FMS is connected.
        public static final boolean killswitch = false;
        // If true, data won't be sent over network even when not connected to FMS
        public static final boolean disableNetworkLogging = false;
        // ONLY ENABLE IN DEV (this *should* be overwritten when connected to FMS, but that's untested)
        public static final boolean disableDatalog = true;
        // Prefix in NetworkTables, must end with a '/'
        public static final String tabPrefix = "UltraLog/";
        // How often to re-check if the FMS is connected (and disable network logging if so)
        public static final double fmsCheckDelay = 1000;
    }

    public static class ClawConstants {
        public static final int beambreakChannel = 0;
        public static final int topMotorId = 0;
        public static final int bottomMotorId = 0;
    }

    public static class PivotConstants {
        public static final int motorID = 0;
        // TODO figure out real values
        public static final Angle floorAngle = Degrees.of(0);
        public static final Angle onCoralAngle = Degrees.of(0);
        public static final Angle reefAngle = Degrees.of(0);
        public static final Angle bargeAngle = Degrees.of(0);

        public static final Angle maxAngle = Degrees.of(5);
        public static final Angle minAngle = Degrees.of(0);

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kG = 0.0;
    }

    public static class ElevatorConstants {
        public static final double floorHeight = 0;
        public static final double onCoralHeight = 0;
        public static final double reefOneHeight = 0;
        public static final double reefTwoHeight = 0;
        public static final double bargeHeight = 0;
        // TODO figure out real conversion and heights
        public static final double inchesPerRotation = 20;

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;    
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kG = 0.0;
    }
}
