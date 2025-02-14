package frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.units.measure.Voltage;

public interface PivotIO {
    @AutoLog
    public class PivotInputs {
        public double temperature;
        public double velocity;
        public double statorCurrent;
        public boolean motorAlive;
        public double position;

        public double pidOutput;
        public double ffOutput;

        public double encoderPosition;
    }

    public default void updateInputs(PivotInputs inputs) {}

    public default void runMotorDutyCycle(double speed) {}

    public default void runMotorVoltageOut(Voltage volts) {}

    public default void runMotorControl(ControlRequest control) {}
}
