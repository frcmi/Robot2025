package frc.lib.ultralogger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class UltraSupplierLog {
  private DoubleSupplier supplier;
  private UltraDoubleLog logEntry;

  public UltraSupplierLog(String name, Supplier<Double> tempSupplier) {
    logEntry = new UltraDoubleLog(name);
    this.supplier = tempSupplier::get;
  }

  public UltraSupplierLog(String name, DoubleSupplier tempSupplier) {
    logEntry = new UltraDoubleLog(name);
    this.supplier = tempSupplier;
  }

  public void update() {
    logEntry.update(supplier.getAsDouble());
  }
}
