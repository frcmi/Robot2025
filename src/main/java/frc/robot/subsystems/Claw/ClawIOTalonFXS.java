package frc.robot.subsystems.Claw;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClawIOTalonFXS implements ClawIO {
    private final DigitalInput beambreak;
    private final TalonFXS clawMotor;

    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Temperature> motorTemperature;
    private final StatusSignal<Current> motorStatorCurrent;

    public ClawIOTalonFXS(int beambreakID, int talonID) {
        beambreak = new DigitalInput(beambreakID);
        clawMotor = new TalonFXS(talonID);

        TalonFXSConfiguration configuration = new TalonFXSConfiguration();
        configuration.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
        clawMotor.getConfigurator().apply(configuration);

        motorVelocity = clawMotor.getVelocity();
        motorTemperature = clawMotor.getDeviceTemp();
        motorStatorCurrent = clawMotor.getStatorCurrent();

        StatusSignal.setUpdateFrequencyForAll(
            Hertz.of(50), 
            motorVelocity, 
            motorTemperature, 
            motorStatorCurrent
        );
    }

    @Override
    public void updateInputs(ClawInputs inputs) {
        inputs.beambreakState = beambreak.get();
        inputs.motorAlive = clawMotor.isAlive();
        inputs.velocity = motorVelocity.getValueAsDouble();
        inputs.temperature = motorTemperature.getValueAsDouble();
        inputs.statorCurrent = motorStatorCurrent.getValueAsDouble();
    }

    @Override
    public void runMotorDutyCycle(double speed) {
        clawMotor.set(speed);
    }
}
