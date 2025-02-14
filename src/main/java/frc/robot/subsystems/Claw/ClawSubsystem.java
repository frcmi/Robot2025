package frc.robot.subsystems.Claw;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import org.littletonrobotics.junction.Logger;

public class ClawSubsystem extends SubsystemBase {
  private final Alert nomotorAlert = new Alert("Claw motor not detected!", AlertType.kError);
  private final ClawIO clawIO;
  private final ClawInputsAutoLogged inputs = new ClawInputsAutoLogged();

  public ClawSubsystem(ClawIO clawIO) {
    this.clawIO = clawIO;
  }

  public boolean isCurrentSpiked() {
    return inputs.statorCurrent > 1.0;
  }

  public Command stop() {
    return run(
        () -> {
          clawIO.runMotorDutyCycle(ClawConstants.stopSpeed);
        });
  }

  public Command runMotor(double speed) {
    return run(
        () -> {
          clawIO.runMotorDutyCycle(speed);
        });
  }

  public Command intakeWithBeambreak() {
    return runMotor(ClawConstants.intakeSpeed)
        .until(
            () -> {
              return !inputs.beambreakState;
            })
        .andThen(stop());
  }

  public Command intake() {
    return runMotor(ClawConstants.intakeSpeed);
  }

  public Command shootWithBeambreak() {
    return runMotor(ClawConstants.shootSpeed)
        .until(
            () -> {
              return inputs.beambreakState;
            })
        .andThen(stop());
  }

  public Command shoot() {
    return runMotor(ClawConstants.shootSpeed);
  }

  @Override
  public void periodic() {
    clawIO.updateInputs(inputs);
    Logger.processInputs("Claw", inputs);
    nomotorAlert.set(inputs.motorAlive);
  }
}
