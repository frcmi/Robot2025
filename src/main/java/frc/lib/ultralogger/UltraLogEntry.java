package frc.lib.ultralogger;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.TelemetryConstants;

public interface UltraLogEntry<T> {
    String logName = "";

    void update(T item);

    static boolean disableNetworkTableLogs() {
        return TelemetryConstants.disableNetworkLogging || DriverStation.isFMSAttached();
    }

    static boolean disableDatalog() {
        return TelemetryConstants.disableDatalog && !DriverStation.isFMSAttached();
    }
}
