package frc.robot;

import edu.wpi.first.math.util.Units;

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
        public static final double fmsCheckDelay = Units.secondsToMilliseconds(1);
    }
}
