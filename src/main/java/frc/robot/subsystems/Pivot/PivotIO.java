package frc.robot.subsystems.Pivot;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public class PivotInputs {
    public double temperature;
    public double velocity;
    public double statorCurrent;
    public boolean motorAlive;
    public double position;
    public double setPoint;

    public double pidOutput;
    public double ffOutput;

    public double encoderPosition;
  }

  public default void updateInputs(PivotInputs inputs) {}

  public default void runVoltage(Voltage volts) {}
}
