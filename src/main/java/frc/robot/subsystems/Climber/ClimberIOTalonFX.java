package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX climberMotor;

  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Temperature> motorTemperature;
  private final StatusSignal<Current> motorStatorCurrent;

  public ClimberIOTalonFX(int motorID) {
    climberMotor = new TalonFX(motorID);

    climberMotor.setNeutralMode(NeutralModeValue.Brake);

    motorVelocity = climberMotor.getVelocity();
    motorTemperature = climberMotor.getDeviceTemp();
    motorStatorCurrent = climberMotor.getStatorCurrent();
  }

  @Override
  public void updateInputs(ClimberInputs inputs) {
    inputs.motorAlive = climberMotor.isAlive();
    inputs.velocity = motorVelocity.getValueAsDouble();
    inputs.temperature = motorTemperature.getValueAsDouble();
    inputs.statorCurrent = motorStatorCurrent.getValueAsDouble();
  }

  @Override
  public void runMotorDutyCycle(double speed) {
    climberMotor.set(speed);
  }
}
