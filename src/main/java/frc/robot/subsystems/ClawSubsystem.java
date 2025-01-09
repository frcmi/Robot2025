package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.ultralogger.UltraBooleanLog;
import frc.lib.ultralogger.UltraTempLog;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
  private final TalonFX topMotor = new TalonFX(ClawConstants.topMotorId);
  private final TalonFX bottomMotor = new TalonFX(ClawConstants.bottomMotorId);
  private final UltraBooleanLog beambreakPublisher = new UltraBooleanLog("Claw/Beambreak");
  private final UltraTempLog topMotorSpeedPublisher = new UltraTempLog("Claw/Top motor speed", topMotor.getVelocity()::getValueAsDouble);
  private final UltraTempLog bottomMotorSpeedPublisher = new UltraTempLog("Claw/Bottom motor speed", bottomMotor.getVelocity()::getValueAsDouble);
  private final UltraTempLog topMotorTempPublisher = new UltraTempLog("Claw/Top motor temperature", topMotor.getDeviceTemp()::getValueAsDouble);
  private final UltraTempLog bottomMotorTempPublisher = new UltraTempLog("Claw/Bottom motor temperature", bottomMotor.getDeviceTemp()::getValueAsDouble);

  public final DigitalInput beambreak = new DigitalInput(ClawConstants.beambreakChannel);

  public ClawSubsystem() {
    topMotor.setNeutralMode(NeutralModeValue.Brake);
    bottomMotor.setNeutralMode(NeutralModeValue.Brake);

    // TODO: Change oppose master direction based on final claw design
    bottomMotor.setControl(new Follower(topMotor.getDeviceID(), true));

    setDefaultCommand(stop());
  }

  public Command stop() {
    return run(() -> {
        topMotor.set(0.0);
    });
  }

  public Command runMotor(double speed){
    return run(() -> {
        topMotor.set(speed);
    });
  }

  public Command intake(){
    return runMotor(1).until(beambreak::get).andThen(stop());

  }

  public Command shoot() {
    return runMotor(-1).until(() -> !beambreak.get()).andThen(stop());
  }

  @Override
  public void periodic() {
    beambreakPublisher.update(beambreak.get());

    topMotorSpeedPublisher.update();
    bottomMotorSpeedPublisher.update();
    topMotorTempPublisher.update();
    bottomMotorTempPublisher.update();
  }
}