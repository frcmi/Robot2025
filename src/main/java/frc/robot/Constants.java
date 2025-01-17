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
        public static final int beambreakChannel = 2;
        public static final int topMotorId = 0;
        public static final int bottomMotorId = 0;
    }

    public static class PivotConstants {
        public static final int motorID = 40;
        // TODO figure out real values
        public static final Angle floorAngle = Degrees.of(10);
        public static final Angle onCoralAngle = Degrees.of(40);
        public static final Angle reefAngle = Degrees.of(90);
        public static final Angle bargeAngle = Degrees.of(190);

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
        public static final int absoluteEncoderChannel = 0;
        public static final int limitSwitchChannel = 1;

        public static final Angle absoluteEncoderOffset = Degrees.of(0);

        public static final double floorHeight = 0;
        public static final double onCoralHeight = 0.3;
        public static final double reefOneHeight = 0.8;
        public static final double reefTwoHeight = 1.0;
        public static final double bargeHeight = 2;
        // TODO: figure out real conversion and heights
        public static final double inchesPerRotation = 20;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;    
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kG = 1.0;

        public static final double rotationsPerMeter = 2; // ROTATIONS OF OUTPUT GEAR, NOT MOTOR 
        public static final double gearRatio = 8.0/1.0;
        public static final double elevatorInertia = 0.001; // In Kg Meters^2 
        public static final double minElevatorHeight = 0.2; // FOR SIM!!!
        public static final double gravity = 2;
    }
}
