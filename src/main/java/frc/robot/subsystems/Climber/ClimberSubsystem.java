package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  public final Alert noclimberAlert = new Alert("Climber motor not detected!", AlertType.kError);
  public final ClimberIO climberIO;
  public final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

  public ClimberSubsystem(ClimberIO climberIO) {
    setDefaultCommand(stop());
    this.climberIO = climberIO;
  }

  public Command stop() {
    return run(() -> {
        climberIO.runMotorDutyCycle(0.0);
    });
  }

  public Command runMotor(double speed){
    return run(() -> {
      climberIO.runMotorDutyCycle(speed);
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
    climberIO.updateInputs(inputs);
    // noclimberAlert.set();
    // climberMotorSpeedPublisher.update();
    // climberMotorTempPublisher.update();
  }
}