package frc.robot.subsystems.Pivot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotIOTalonFX implements PivotIO {
  private final TalonFX pivotMotor;
  private final DutyCycleEncoder encoder;

  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Temperature> motorTemperature;
  private final StatusSignal<Current> motorStatorCurrent;
  private final StatusSignal<Angle> motorPosition;

  private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(true);

  public PivotIOTalonFX(int motorID, int encoderID) {
    pivotMotor = new TalonFX(motorID);
    encoder = new DutyCycleEncoder(encoderID);

    pivotMotor.setNeutralMode(NeutralModeValue.Brake);

    motorVelocity = pivotMotor.getVelocity();
    motorTemperature = pivotMotor.getDeviceTemp();
    motorStatorCurrent = pivotMotor.getStatorCurrent();
    motorPosition = pivotMotor.getPosition();

    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotMotor.getConfigurator().apply(configuration);
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    inputs.motorAlive = pivotMotor.isAlive();
    inputs.velocity = motorVelocity.getValueAsDouble();
    inputs.temperature = motorTemperature.getValueAsDouble();
    inputs.statorCurrent = motorStatorCurrent.getValueAsDouble();
    inputs.position = motorPosition.getValueAsDouble();

    inputs.encoderPosition = encoder.get();
  }

  @Override
  public void runVoltage(Voltage volts) {
    pivotMotor.setControl(voltageControl.withOutput(volts));
  }
}
