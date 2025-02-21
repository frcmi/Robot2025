package frc.robot.subsystems.Pivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ultralogger.UltraDoubleLog;
import frc.robot.Constants.BotType;
import frc.robot.Constants.PivotConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class PivotSubsystem extends SubsystemBase {
  private final Alert limitPassedAlert =
      new Alert("Pivot motor limit has been exceeded!", AlertType.kError);
  private final Alert nopivotAlert = new Alert("Pivot motor not detected!", AlertType.kError);

  private final PivotIO pivotIO;
  private final PivotInputsAutoLogged inputs = new PivotInputsAutoLogged();

  // MechanismLigament2d pivotLigament2d;
  // TalonFXSimState talonFXSim = pivotMotor.getSimState();
  // private static final double kGearRatio = 10.0;
  // private final DCMotorSim m_motorSimModel = new DCMotorSim(
  // LinearSystemId.createDCMotorSystem(
  //   DCMotor.getKrakenX60Foc(1), 0.001, kGearRatio
  // ),
  //   DCMotor.getKrakenX60Foc(1)
  // );

  private final UltraDoubleLog setpointPublisher = new UltraDoubleLog("Pivot/Setpoint Rotations");
  private final UltraDoubleLog pidPublisher = new UltraDoubleLog("Pivot/PID Output");
  private final UltraDoubleLog ffPublisher = new UltraDoubleLog("Pivot/FF Output");

  // private final UltraDoubleLog encoderPublisher = new UltraDoubleLog("Pivot/Encoder Rotations");
  // private final UltraDoubleLog velPublisher = new UltraDoubleLog("Pivot/Velocity");
  private ProfiledPIDController pid;

  private ArmFeedforward feedforward;

  LoggedMechanismLigament2d pivotMechanism2d =
      new LoggedMechanismLigament2d("Pivot", PivotConstants.length, 0);
  private Angle setAngle = Rotations.of(0);

  public PivotSubsystem(BotType botType, PivotIO pivotIO) {
    if (botType == BotType.MAIN_BOT) {
      pid =
          new ProfiledPIDController(
              PivotConstants.kRealBotP,
              PivotConstants.kRealBotI,
              PivotConstants.kRealBotD,
              new Constraints(PivotConstants.maxVelocity, PivotConstants.maxAccel));
      feedforward =
          new ArmFeedforward(
              PivotConstants.kRealBotS,
              PivotConstants.kRealBotG,
              PivotConstants.kRealBotV,
              PivotConstants.kRealBotA);
    } else if (botType == BotType.SIM_BOT) {
      pid =
          new ProfiledPIDController(
              PivotConstants.kSimulationBotP,
              PivotConstants.kSimulationBotI,
              PivotConstants.kSimulationBotD,
              new Constraints(PivotConstants.maxVelocity, PivotConstants.maxAccel));
      feedforward =
          new ArmFeedforward(
              PivotConstants.kSimulationBotS,
              PivotConstants.kSimulationBotG,
              PivotConstants.kSimulationBotV,
              PivotConstants.kSimulationBotA);
    }
    pid.setTolerance(Degrees.of(3.5).in(Radians));
    this.pivotIO = pivotIO;
    setDefaultCommand(this.setHoldAngle());
  }

  public void addLigament(LoggedMechanismLigament2d elevatorMechanism2d) {
    elevatorMechanism2d.append(pivotMechanism2d);
  }

  public Command holdCurrentPosition() {
    return run(
        () -> {
          double ff = feedforward.calculate(getEncoderAngle().in(Radians), 0);

          pidPublisher.update(0.0);
          ffPublisher.update(ff);

          pivotIO.runVoltage(Volts.of(ff));
        });
  }

  public Command setHoldAngle() {
    return runOnce(
        () -> {
          double setpoint = setAngle.in(Radians);
          setpointPublisher.update(setAngle.in(Rotations));

          double currentAngle = getEncoderAngle().in(Radians);

          double pidOut = pid.calculate(currentAngle, setpoint);
          pidPublisher.update(pidOut);

          double ff = feedforward.calculate(currentAngle, pid.getSetpoint().velocity);
          ffPublisher.update(ff);

          pivotIO.runVoltage(Volts.of(pidOut + ff));
        });
  }

  public Command goToAngle(int level) {
    switch (level) {
      case 0:
        return goToFloorAngle();
      case 1:
        return goToOnCoralAngle();
      case 2:
        return goToReefOneAngle();
      case 3:
        return goToReefTwoAngle();
      case 4:
        return goToBargeAngle();
      default:
        return goToFloorAngle();
    }
  }

  public Command goToFloorAngle() {
    return Commands.runOnce(
        () -> {
          setAngle = PivotConstants.floorAngle;
        });
  }

  public Command goToOnCoralAngle() {
    return Commands.runOnce(
        () -> {
          setAngle = PivotConstants.onCoralAngle;
        });
  }

  public Command goToReefOneAngle() {
    return Commands.runOnce(
        () -> {
          setAngle = PivotConstants.reefOneAngle;
        });
  }

  public Command goToReefTwoAngle() {
    return Commands.runOnce(
        () -> {
          setAngle = PivotConstants.reefTwoAngle;
        });
  }

  public Command goToBargeAngle() {
    return Commands.runOnce(
        () -> {
          setAngle = PivotConstants.bargeAngle;
        });
  }

  public Angle getEncoderAngle() {
    if (Robot.isReal()) {
      double encoderValue = inputs.encoderPosition - 0.75;
      if (encoderValue <= -0.2) {
        encoderValue += 1;
      }
      return Rotations.of(encoderValue);
    }
    return Rotations.of(inputs.encoderPosition);
  }

  public void sysIDLog() {
    double encoderValue = getEncoderAngle().in(Rotations);

    SignalLogger.writeDouble("Pivot Angle", encoderValue);
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    pivotMechanism2d.setAngle(Rotations.of(inputs.position).in(Degrees) - 90);

    if (inputs.position <= PivotConstants.minAngle.in(Rotations)
        && inputs.position >= PivotConstants.maxAngle.in(Rotations)) {
      pivotIO.runVoltage(Volts.of(0));
      limitPassedAlert.set(true);
    }

    nopivotAlert.set(!inputs.motorAlive);
  }
}
