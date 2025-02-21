package frc.robot.subsystems.Elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public class ElevatorInputs {
    public double leftTemperature;
    public double leftVelocity;
    public double leftPosition;
    public double leftStatorCurrent;
    public double leftSetPoint;
    public boolean leftMotorAlive;

    public double rightTemperature;
    public double rightPosition;
    public boolean rightMotorAlive;

    public boolean upperLimitSwitchState;
    public boolean lowerLimitSwitchState;
  }

  public default void updateInputs(ElevatorInputs inputs) {}

  public default void runVoltage(Voltage volts) {}

  public default void runPositionControl(Angle position) {}
}
