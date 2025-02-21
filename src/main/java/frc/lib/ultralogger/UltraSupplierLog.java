package frc.lib.ultralogger;

import com.ctre.phoenix6.StatusSignal;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class UltraSupplierLog {
  private DoubleSupplier supplier;
  private UltraDoubleLog logEntry;

  public UltraSupplierLog(String name, StatusSignal doubleSupplier) {
    logEntry = new UltraDoubleLog(name);
    this.supplier =
        () -> {
          StatusSignal.refreshAll(doubleSupplier);
          return doubleSupplier.getValueAsDouble();
        };
  }

  public UltraSupplierLog(String name, Supplier<Double> doubleSupplier) {
    logEntry = new UltraDoubleLog(name);
    this.supplier = doubleSupplier::get;
  }

  public UltraSupplierLog(String name, DoubleSupplier doubleSupplier) {
    logEntry = new UltraDoubleLog(name);
    this.supplier = doubleSupplier;
  }

  public void update() {
    logEntry.update(supplier.getAsDouble());
  }
}
