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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ultralogger.UltraDoubleLog;
import frc.robot.Constants.PivotConstants;
import org.littletonrobotics.junction.Logger;

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
  private ProfiledPIDController pid =
      new ProfiledPIDController(
          PivotConstants.kP,
          PivotConstants.kI,
          PivotConstants.kD,
          new Constraints(PivotConstants.maxVelocity, PivotConstants.maxAccel));
  private ArmFeedforward feedforward =
      new ArmFeedforward(
          PivotConstants.kS, PivotConstants.kG, PivotConstants.kV, PivotConstants.kA);

  public PivotSubsystem(PivotIO pivotIO) {
    // pivotLigament2d = elevatorLigament.append(new MechanismLigament2d("wrist", 0.5, 90, 6, new
    // Color8Bit(Color.kPurple)));
    pid.setTolerance(Degrees.of(3.5).in(Radians));
    this.pivotIO = pivotIO;
    setDefaultCommand(this.holdPosition());
  }

  public Command holdPosition() {
    return run(
        () -> {
          double ff = feedforward.calculate(getEncoderAngle().in(Radians), 0);

          pidPublisher.update(0.0);
          ffPublisher.update(ff);

          pivotIO.runMotorVoltageOut(Volts.of(ff));
        });
  }

  public Command setAngle(Angle angle) {
    double setpoint = angle.in(Radians);
    return run(() -> {
          setpointPublisher.update(angle.in(Rotations));

          double currentAngle = getEncoderAngle().in(Radians);

          double pidOut = pid.calculate(currentAngle, setpoint);
          pidPublisher.update(pidOut);

          double ff = feedforward.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
          ffPublisher.update(ff);

          pivotIO.runMotorVoltageOut(Volts.of(pidOut + ff));
        })
        .until(pid::atGoal);
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
    return setAngle(PivotConstants.floorAngle);
  }

  public Command goToOnCoralAngle() {
    return setAngle(PivotConstants.onCoralAngle);
  }

  public Command goToReefOneAngle() {
    return setAngle(PivotConstants.reefOneAngle);
  }

  public Command goToReefTwoAngle() {
    return setAngle(PivotConstants.reefTwoAngle);
  }

  public Command goToBargeAngle() {
    return setAngle(PivotConstants.bargeAngle);
  }

  public Angle getEncoderAngle() {
    double encoderValue = inputs.encoderPosition - 0.75;
    if (encoderValue <= -0.2) {
      encoderValue += 1;
    }

    return Rotations.of(encoderValue);
  }

  public void sysIDLog() {
    double encoderValue = getEncoderAngle().in(Rotations);

    SignalLogger.writeDouble("Pivot Angle", encoderValue);
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    if (inputs.position <= PivotConstants.minAngle.in(Rotations)
        && inputs.position >= PivotConstants.maxAngle.in(Rotations)) {
      pivotIO.runMotorVoltageOut(Volts.of(0));
      limitPassedAlert.set(true);
    }
    nopivotAlert.set(!inputs.motorAlive);
    // if (Robot.isReal()) {
    //     pivotLigament2d.setAngle(pivotMotor.getPosition().getValue().in(Degrees));
    // } else if (Robot.isSimulation()) {
    //
    // pivotLigament2d.setAngle(pivotMotor.getPosition().getValue().minus(Degrees.of(90)).in(Degrees));
    // }
  }

  // @Override
  // public void simulationPeriodic() {
  //     talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
  //     var motorVoltage = talonFXSim.getMotorVoltageMeasure();
  //     m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
  //     m_motorSimModel.update(0.020);
  //     talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
  //     talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
  // }
}
