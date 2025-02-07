package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.ultralogger.UltraBooleanLog;
import frc.lib.ultralogger.UltraSupplierLog;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ClimberCostansts;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climberMotor = new TalonFX(ClimberCostansts.climberMotorID);
  private final UltraSupplierLog climberMotorSpeedPublisher = new UltraSupplierLog("Climber motor speed", climberMotor.getVelocity()::getValueAsDouble);
  private final UltraSupplierLog climberMotorTempPublisher = new UltraSupplierLog("Climber motor temperature", climberMotor.getDeviceTemp()::getValueAsDouble);
  Alert noclimberAlert = new Alert("Climber motor not detected!", AlertType.kError);

  public ClimberSubsystem() {
    climberMotor.setNeutralMode(NeutralModeValue.Brake);

    setDefaultCommand(stop());
  }

  public Command stop() {
    return run(() -> {
        climberMotor.set(0.0);
    });
  }

  public Command runMotor(double speed){
    return run(() -> {
        climberMotor.set(speed);
    });
  }

  public Command runClimberup(){
    return run(() -> {
      runMotor(0.4).withTimeout(1.5).andThen(stop());
    });
  }

  public Command runClimberdown(){
    return run(() -> {
      runMotor(-0.4).withTimeout(1.5).andThen(stop());
    });
  }


  @Override
  public void periodic() {
    noclimberAlert.set(!climberMotor.isAlive());
    climberMotorSpeedPublisher.update();
    climberMotorTempPublisher.update();

  }
}