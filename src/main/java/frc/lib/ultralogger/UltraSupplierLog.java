package frc.lib.ultralogger;

import java.lang.reflect.InvocationTargetException;
import java.util.function.Supplier;

public class UltraSupplierLog<T> {
    private Supplier<T> tempSupplier;
    private UltraLogEntry<T> logEntry;

    public UltraSupplierLog(Class<UltraLogEntry<T>> e, String name, Supplier<T> tempSupplier) {
        try {
            logEntry = e.getDeclaredConstructor().newInstance(name);
        } catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException
                | NoSuchMethodException | SecurityException e1) {
            e1.printStackTrace();
        }
        this.tempSupplier = tempSupplier::get;
    }
    
    // public UltraTempLog(String name, DoubleSupplier tempSupplier) {
    //     logEntry = new UltraDoubleLog(name);
    //     this.tempSupplier = tempSupplier;
    // }

    public void update() {
        logEntry.update(tempSupplier.get());
    }
}
