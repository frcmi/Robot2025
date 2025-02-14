package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public class ClimberInputs {
        public double temperature;
        public double velocity;
        public double statorCurrent;
        public boolean motorAlive;
    }

    public default void updateInputs(ClimberInputs inputs) {}

    public default void runMotorDutyCycle(double speed) {}
}
