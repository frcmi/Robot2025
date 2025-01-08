package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.ultralogger.UltraBooleanLog;

public class ClawSubsystem extends SubsystemBase {

    // get ID later
    private final TalonFX intake1 = new TalonFX(0);
    private final TalonFX intake2 = new TalonFX(1);
    private final UltraBooleanLog beambreakPublisher = new UltraBooleanLog("Claw Beambreak");

    private final Boolean Beambreak;
    private final LEDSubsystem ledSubsystem;

  public ClawSubsystem(Boolean Beambreak, LEDSubsystem ledSubsystem) {
    this.Beambreak = Beambreak;
    this.ledSubsystem = ledSubsystem;
    intake1.setNeutralMode(NeutralModeValue.Brake);
    intake2.setNeutralMode(NeutralModeValue.Brake);

    setDefaultCommand(stop());
  }

  public Command stop() {
    return run(() -> {
        intake1.set(0.0);
        intake2.set(0.0);
    });
  }

  public Command runMotor(double speed){
    return run(() -> {
        intake1.set(speed);
        intake2.set(speed);
    });
  }

  public Command intake(){
    return runMotor(1).until(() -> !Beambreak).andThen(stop());

  }

  public Command shoot() {
    return runMotor(-1).until(() -> Beambreak).andThen(stop());
  }

  





  @Override
  public void periodic() {
    beambreakPublisher.update(Beambreak);
  }

  @Override
  public void simulationPeriodic() {

  }
}