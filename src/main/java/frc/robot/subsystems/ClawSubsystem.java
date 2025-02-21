package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.ultralogger.UltraBooleanLog;
import frc.lib.ultralogger.UltraSupplierLog;
import frc.robot.Constants.BotType;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
  private final TalonFXS intakeMotor = new TalonFXS(ClawConstants.motorControllerID);
  private final UltraBooleanLog beambreakPublisher = new UltraBooleanLog("Claw/Beambreak");
  private final UltraSupplierLog topMotorSpeedPublisher = new UltraSupplierLog("Claw/Intake motor speed", intakeMotor.getVelocity()::getValueAsDouble);
  private final UltraSupplierLog topMotorTempPublisher = new UltraSupplierLog("Claw/Intake motor temperature", intakeMotor.getExternalMotorTemp()::getValueAsDouble);
  Alert nomotorAlert = new Alert("Claw motor not detected!", AlertType.kError);

  public final DigitalInput beambreak = new DigitalInput(ClawConstants.beambreakChannel);

  public ClawSubsystem(BotType bot) {
    TalonFXSConfiguration configure = new TalonFXSConfiguration();
    configure.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    intakeMotor.getConfigurator().apply(configure);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);

    setDefaultCommand(stop());
  }

  public boolean isCurrentSpiked() {
    return intakeMotor.getStatorCurrent().getValueAsDouble() > 1.0;
}

  public Command stop() {
    
    return run(() -> {
        intakeMotor.set(ClawConstants.stopSpeed);
    });
  }

  public Command runMotor(double speed){
    return run(() -> {
        intakeMotor.set(speed);
    });
  }

  public Command intakeWithBeambreak(){
    return runMotor(ClawConstants.intakeSpeed).until(() -> !beambreak.get()).andThen(stop());
  }
  
  public Command intake() {
    return runMotor(ClawConstants.intakeSpeed);
  }

  public Command shootWithBeambreak() {
    return runMotor(ClawConstants.shootSpeed).until(beambreak::get).andThen(stop());
  }

  public Command shoot() {
    return runMotor(ClawConstants.shootSpeed);
  }

  @Override
  public void periodic() {
    nomotorAlert.set(!intakeMotor.isAlive());
    beambreakPublisher.update(beambreak.get());
    topMotorSpeedPublisher.update();
    topMotorTempPublisher.update();
  }
}