package frc.lib.ultralogger;

import frc.robot.Constants.TelemetryConstants;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.Optional;

public class UltraJSONLog implements UltraLogEntry<String> {
    public String logName;
    private Optional<GenericPublisher> ntPublisher = Optional.empty();
    private Optional<StringLogEntry> datalogPublisher = Optional.empty();
    private double lastCheckedTimestamp = System.currentTimeMillis();
    private boolean errored = false;


    public UltraJSONLog(String name) {
        if (TelemetryConstants.killswitch) {return;}

        this.logName = TelemetryConstants.tabPrefix + name;

        try {
            checkNTFMS(false);
            checkDLFMS();
        } catch (Throwable error) {
            System.err.println("Error in UltraJSONLog constructor, aborting logger:\n" + error);
            errored = true;
        }
    }

    private void checkDLFMS() {
        if (UltraLogEntry.disableDatalog()) {
            return;
        }

        this.datalogPublisher = Optional.of(new StringLogEntry(DataLogManager.getLog(), logName, "", "json"));
    }

    private void checkNTFMS(boolean doTimestamp) {
        if (doTimestamp && System.currentTimeMillis() - lastCheckedTimestamp < TelemetryConstants.fmsCheckDelay) {
            return;
        }

        this.lastCheckedTimestamp = System.currentTimeMillis();

        if (UltraLogEntry.disableNetworkTableLogs()) {
            this.ntPublisher = Optional.empty();
            return;
        }

        if (this.ntPublisher.isEmpty()) {
            this.ntPublisher = Optional.of(NetworkTableInstance.getDefault().getTopic(logName).genericPublish("json"));
        }
    }

    public void update(String item) {
        if (TelemetryConstants.killswitch || errored) {return;}
        try {
            if (item == null) {
                return;
            }

            if (this.datalogPublisher.isPresent()) {
                this.datalogPublisher.get().append(item);
            } else {
                checkDLFMS();
            }

            if (this.ntPublisher.isPresent()) {
                ntPublisher.get().setString(item);
                checkNTFMS(true);
            }
        } catch (Throwable error) {
            System.err.println("Error in UltraJSONLog, aborting logger:\n" + error);
            errored = true;
        }
    }
}
