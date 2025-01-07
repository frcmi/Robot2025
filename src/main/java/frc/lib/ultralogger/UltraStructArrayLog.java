package frc.lib.ultralogger;

import edu.wpi.first.util.struct.Struct;
import frc.robot.Constants.TelemetryConstants;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.Optional;

public class UltraStructArrayLog<T> implements UltraLogEntry<T[]> {
    public String logName;
    private Optional<StructArrayPublisher<T>> ntPublisher = Optional.empty();
    private Optional<StructArrayLogEntry<T>> datalogPublisher = Optional.empty();
    private double lastCheckedTimestamp = System.currentTimeMillis();
    private final Struct<T> struct;
    private boolean errored = false;

    public UltraStructArrayLog(String name, Struct<T> struct) {
        if (TelemetryConstants.killswitch) {
            // SAFETY: if killswitch is true `struct` will never be read.
            this.struct = null;
            return;
        }

        this.logName = TelemetryConstants.tabPrefix + name;
        this.struct = struct;

        try {
            if (struct == null) {
                throw new Exception("struct cannot be null");
            }

            checkNTFMS(false);
            checkDLFMS();
        } catch (Throwable error) {
            System.err.println("Error in UltraStructArrayLog constructor, aborting logger:\n" + error);
            errored = true;
        }
    }

    private void checkDLFMS() {
        if (UltraLogEntry.disableDatalog()) {
            return;
        }

        this.datalogPublisher = Optional.of(StructArrayLogEntry.create(DataLogManager.getLog(), logName, struct));
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
            this.ntPublisher = Optional.of(NetworkTableInstance.getDefault().getStructArrayTopic(logName, struct).publish());
        }
    }

    public void update(T[] items) {
        if (TelemetryConstants.killswitch || errored) {return;}
        try {
            if (items == null) {
                return;
            }

            for (T item : items) {
                if (item == null) {
                    return;
                }
            }

            if (this.datalogPublisher.isPresent()) {
                this.datalogPublisher.get().append(items);
            } else {
                checkDLFMS();
            }

            if (this.ntPublisher.isPresent()) {
                ntPublisher.get().set(items);
                checkNTFMS(true);
            }
        } catch (Throwable error) {
            System.err.println("Error in UltraStructLog, aborting logger:\n" + error);
            errored = true;
        }
    }
}
