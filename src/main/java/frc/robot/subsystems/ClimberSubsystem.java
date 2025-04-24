package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.ultralogger.UltraSupplierLog;
import frc.robot.Constants.ClimberCostansts;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climberMotor = new TalonFX(ClimberCostansts.climberMotorID);
  private final UltraSupplierLog climberMotorSpeedPublisher = new UltraSupplierLog("Climber motor speed", climberMotor.getVelocity()::getValueAsDouble);
  private final UltraSupplierLog climberMotorTempPublisher = new UltraSupplierLog("Climber motor temperature", climberMotor.getDeviceTemp()::getValueAsDouble);
  Alert noclimberAlert = new Alert("Climber motor not detected!", AlertType.kError);
  TorqueCurrentFOC control = new TorqueCurrentFOC(Amps.of(-17)).withOverrideCoastDurNeutral(false).withMaxAbsDutyCycle(0.7);
  DutyCycleOut reverse = new DutyCycleOut(0.4).withEnableFOC(true);

  public ClimberSubsystem() {
    SmartDashboard.putNumber("Climber Torque", 17);
    climberMotor.setNeutralMode(NeutralModeValue.Brake);

    setDefaultCommand(stop());
  }

  public Command stop() {
    return run(() -> {
        climberMotor.setControl(new NeutralOut());
    });
  }

  public Command runMotorDown(double torque){
    return run(() -> {
        climberMotor.setControl(control.withOutput(-torque));
    });
  }

  public Command runMotorUp(double speed){
    return run(() -> {
        climberMotor.setControl(reverse.withOutput(speed));
    });
  }

  public Command runClimberup(){
    return runMotorUp(0.4).andThen(stop());
  }

  public Command runClimberupAuto(){
    return runMotorUp(0.6).andThen(stop());
  }

  public Command runClimberdown(){
    return runMotorDown(17).andThen(stop());
  }


  @Override
  public void periodic() {
    noclimberAlert.set(!climberMotor.isAlive());
    climberMotorSpeedPublisher.update();
    climberMotorTempPublisher.update();

  }
}