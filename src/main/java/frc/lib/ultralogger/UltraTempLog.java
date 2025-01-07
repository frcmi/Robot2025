package frc.lib.ultralogger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class UltraTempLog {
    private DoubleSupplier tempSupplier;
    private UltraDoubleLog logEntry;

    public UltraTempLog(String name, Supplier<Double> tempSupplier) {
        logEntry = new UltraDoubleLog(name);
        this.tempSupplier = tempSupplier::get;
    }
    
    public UltraTempLog(String name, DoubleSupplier tempSupplier) {
        logEntry = new UltraDoubleLog(name);
        this.tempSupplier = tempSupplier;
    }

    public void update() {
        logEntry.update(tempSupplier.getAsDouble());
    }
}
