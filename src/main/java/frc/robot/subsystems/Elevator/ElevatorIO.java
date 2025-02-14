package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

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
        public double rightVelocity;
        public double rightPosition;
        public double rightStatorCurrent;
        public boolean rightMotorAlive;

        public boolean limitSwitchState;
    }

    public default void updateInputs(ElevatorInputs inputs) {}

    public default void runVoltage(Voltage volts) {}
}
