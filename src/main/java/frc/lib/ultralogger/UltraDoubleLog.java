package frc.lib.ultralogger;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants.TelemetryConstants;
import java.util.Optional;

public class UltraDoubleLog implements UltraLogEntry<Double> {
  public String logName;
  private Optional<DoublePublisher> ntPublisher = Optional.empty();
  private Optional<DoubleLogEntry> datalogPublisher = Optional.empty();
  private double lastCheckedTimestamp = System.currentTimeMillis();
  private boolean errored = false;

  public UltraDoubleLog(String name) {
    if (TelemetryConstants.killswitch) {
      return;
    }

    this.logName = TelemetryConstants.tabPrefix + name;

    try {
      checkNTFMS(false);
      checkDLFMS();
    } catch (Throwable error) {
      System.err.println("Error in UltraDoubleLog constructor, aborting logger:\n" + error);
      errored = true;
    }
  }

  private void checkDLFMS() {
    if (UltraLogEntry.disableDatalog()) {
      return;
    }

    this.datalogPublisher = Optional.of(new DoubleLogEntry(DataLogManager.getLog(), logName));
  }

  private void checkNTFMS(boolean doTimestamp) {
    if (doTimestamp
        && System.currentTimeMillis() - lastCheckedTimestamp < TelemetryConstants.fmsCheckDelay) {
      return;
    }

    this.lastCheckedTimestamp = System.currentTimeMillis();

    if (UltraLogEntry.disableNetworkTableLogs()) {
      this.ntPublisher = Optional.empty();
      return;
    }

    if (this.ntPublisher.isEmpty()) {
      this.ntPublisher =
          Optional.of(NetworkTableInstance.getDefault().getDoubleTopic(logName).publish());
    }
  }

  public void update(Double item) {
    if (TelemetryConstants.killswitch || errored) {
      return;
    }
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
      System.err.println("Error in UltraDoubleLog, aborting logger:\n" + error);
      errored = true;
    }
  }
}
