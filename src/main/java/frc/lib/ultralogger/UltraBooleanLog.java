package frc.lib.ultralogger;

import frc.robot.Constants.TelemetryConstants;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.Optional;

public class UltraBooleanLog implements UltraLogEntry<Boolean> {
    public String logName;
    private Optional<BooleanPublisher> ntPublisher = Optional.empty();
    private Optional<BooleanLogEntry> datalogPublisher = Optional.empty();
    private double lastCheckedTimestamp = System.currentTimeMillis();
    private boolean errored = false;


    public UltraBooleanLog(String name) {
        if (TelemetryConstants.killswitch) {return;}

        this.logName = TelemetryConstants.tabPrefix + name;

        try {
            checkNTFMS(false);
            checkDLFMS();
        } catch (Throwable error) {
            System.err.println("Error in UltraBooleanLog constructor, aborting logger:\n" + error);
            errored = true;
        }
    }

    private void checkDLFMS() {
        if (UltraLogEntry.disableDatalog()) {
            return;
        }

        this.datalogPublisher = Optional.of(new BooleanLogEntry(DataLogManager.getLog(), logName));
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
            this.ntPublisher = Optional.of(NetworkTableInstance.getDefault().getBooleanTopic(logName).publish());
        }
    }

    public void update(Boolean item) {
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
                ntPublisher.get().set(item);
                checkNTFMS(true);
            }
        } catch (Throwable error) {
            System.err.println("Error in UltraBooleanLog, aborting logger:\n" + error);
            errored = true;
        }
    }
}
