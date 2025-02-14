package frc.robot.subsystems.Claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
  @AutoLog
  public class ClawInputs {
    public double temperature;
    public double velocity;
    public double statorCurrent;
    public boolean beambreakState;
    public boolean motorAlive;
  }

  public default void updateInputs(ClawInputs inputs) {}
  ;

  public default void runMotorDutyCycle(double speed) {}
  ;
}
