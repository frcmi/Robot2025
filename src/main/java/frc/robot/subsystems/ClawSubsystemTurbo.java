package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.ultralogger.UltraBooleanLog;
import frc.lib.ultralogger.UltraSupplierLog;
import frc.robot.Constants;
import frc.robot.Constants.BotType;
import frc.robot.Constants.ClawConstants;
import edu.wpi.first.wpilibj.DriverStation;

public class ClawSubsystemTurbo extends SubsystemBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private final TalonFX intakeMotor = new TalonFX(ClawConstants.motorControllerID);
  private final UltraBooleanLog beambreakPublisher = new UltraBooleanLog("Claw/Beambreak");
  private final UltraSupplierLog topMotorSpeedPublisher = new UltraSupplierLog("Claw/Intake motor speed", intakeMotor.getVelocity()::getValueAsDouble);
  private final UltraSupplierLog topMotorTempPublisher = new UltraSupplierLog("Claw/Intake motor temperature", intakeMotor.getDeviceTemp()::getValueAsDouble);
  Alert notopAlert = new Alert("Top motor not detected!", AlertType.kError);
  Alert nobottomAlert = new Alert("Bottom motor not detected!", AlertType.kError);

  public final TorqueCurrentFOC foc = new TorqueCurrentFOC(Amps.of(327 * ClawConstants.shootSpeed));
  public final DigitalInput beambreak = new DigitalInput(ClawConstants.beambreakChannel);



  public ClawSubsystemTurbo(BotType bot, ElevatorSubsystem elevatorSubsystem) {
    TalonFXConfiguration configure = new TalonFXConfiguration();
    this.elevatorSubsystem = elevatorSubsystem;
    // configure.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    if (bot == BotType.MAIN_BOT) {
      configure.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
    intakeMotor.getConfigurator().apply(configure);

    intakeMotor.setNeutralMode(NeutralModeValue.Brake);

    setDefaultCommand(stop());
  }

  public boolean isCurrentSpiked() {
    return intakeMotor.getStatorCurrent().getValueAsDouble() > 1.0;
}

  public Command stop() {
    return run(() -> {
      SmartDashboard.putBoolean("Claw Spin", false);
      if (DriverStation.isAutonomous() || DriverStation.isTest()) {
        intakeMotor.set(0);
        return;
      }
        intakeMotor.set(ClawConstants.stopSpeed);
    });
  }

  public Command runMotor(ControlRequest speed){
    return run(() -> {
        SmartDashboard.putBoolean("Claw Spin", true);
        intakeMotor.setControl(speed);
    });
  }

  public Command intakeWithBeambreak(){
    return runMotor(new DutyCycleOut(ClawConstants.intakeSpeed)).until(() -> !beambreak.get()).andThen(stop());
  }
  
  public Command intake() {
    return runMotor(new DutyCycleOut(ClawConstants.intakeSpeed));
  }

  public Command shootWithBeambreak() {
    return shoot().until(beambreak::get).andThen(stop());
  }

  public Command shoot() {
    return run(() -> {
      if(elevatorSubsystem.poseToHold == Constants.ElevatorConstants.onCoralHeight) {
        intakeMotor.setControl(new DutyCycleOut(ClawConstants.processorShootSpeed));
      } else {
        intakeMotor.setControl(new DutyCycleOut(ClawConstants.shootSpeed));
      }
    });
  }

  @Override
  public void periodic() {
    notopAlert.set(!intakeMotor.isAlive());
    beambreakPublisher.update(beambreak.get());
    topMotorSpeedPublisher.update();
    topMotorTempPublisher.update();
    SmartDashboard.putNumber("PoseToHold", elevatorSubsystem.poseToHold);
    SmartDashboard.putBoolean("if statement", elevatorSubsystem.poseToHold == Constants.ElevatorConstants.onCoralHeight);
  }
}