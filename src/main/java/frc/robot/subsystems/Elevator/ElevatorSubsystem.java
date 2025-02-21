package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorSubsystem extends SubsystemBase {
  private final ElevatorIO elevatorIO;
  private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

  private final Alert noElevatorAlert = new Alert("Elevator motor not detected!", AlertType.kError);

  private final LoggedMechanism2d windmill = new LoggedMechanism2d(1.5, 3);

  private final LoggedMechanismRoot2d root = windmill.getRoot("elevator", 0.75, 0);
  public final LoggedMechanismLigament2d elevator =
      root.append(new LoggedMechanismLigament2d("elevator", 2, 90, 10, new Color8Bit(255, 0, 0)));

  public ElevatorSubsystem(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
    // simState.Orientation = ChassisReference.CounterClockwise_Positive;
    SmartDashboard.putData("Windmill", windmill);
  }

  public Command extendArm(double rotations) {
    return runOnce(
        () -> {
          elevatorIO.runPositionControl(Rotations.of(rotations));
        });
  }

  public void driveWithVoltage(Voltage volts) {
    elevatorIO.runVoltage(volts);
  }

  public Command zeroElevatorDown() {
    return goToFloorHeightCommand()
        .until(() -> isRotationsAlmostAtZero())
        .andThen(driveWithSlowVoltageDown())
        .until(
            () -> {
              return isAtExtrema() || !inputs.lowerLimitSwitchState;
            })
        .andThen(stop());
  }

  public Command autoHoneDown() {
    return driveWithSlowVoltageDown()
        .until(this::isAtExtrema)
        .andThen(resetPose())
        .andThen(stop().withTimeout(0.1));
  }

  public Command autoHonePose() {
    return driveWithSlowVoltageUp().withTimeout(0.5).andThen(autoHoneDown());
  }

  public boolean isRotationsAlmostAtMax() {
    return inputs.leftPosition >= ElevatorConstants.rotationsBeforeMaxHeight;
  }

  public Command driveWithSlowVoltageUp() {
    return run(() -> driveWithVoltage(Volts.of(ElevatorConstants.slowVoltageUp)));
  }

  // this command will definently be changed due to how the elevator needs to be slowed down
  public Command zeroElevatorUp() {
    return goToBargeHeightCommand()
        .until(() -> isRotationsAlmostAtMax())
        .andThen(driveWithSlowVoltageUp())
        .until(
            () -> {
              return isAtExtrema() || !inputs.upperLimitSwitchState;
            })
        .andThen(stop());
  }

  public Command resetPose() {
    return runOnce(
        () -> {
          elevatorIO.runVoltage(Volts.of(0));
        });
  }

  public Command goToHeight(int level) {
    switch (level) {
      case 0:
        return goToFloorHeightCommand();
      case 1:
        return goToOnCoralHeightCommand();
      case 2:
        return goToReefOneHeightCommand();
      case 3:
        return goToReefTwoHeightCommand();
      case 4:
        return goToBargeHeightCommand();
      default:
        return goToFloorHeightCommand();
    }
  }

  public Command goToFloorHeightCommand() {
    return (extendArm(ElevatorConstants.floorHeight));
  }

  public Command goToOnCoralHeightCommand() {
    return (extendArm(ElevatorConstants.onCoralHeight));
  }

  public Command goToReefOneHeightCommand() {
    return (extendArm(ElevatorConstants.reefOneHeight));
  }

  public Command goToReefTwoHeightCommand() {
    return (extendArm(ElevatorConstants.reefTwoHeight));
  }

  public Command goToBargeHeightCommand() {
    return (extendArm(ElevatorConstants.bargeHeight));
  }

  /** Height is relative to bottom of motor */
  public Distance getElevatorHeight() {
    return Meters.of(
        inputs.leftPosition / ElevatorConstants.rotationsPerMeter
            + ElevatorConstants.minElevatorHeight);
  }

  public boolean isCurrentSpiked() {
    return inputs.leftStatorCurrent > 1.0;
  }

  public boolean isRotationsAlmostAtZero() {
    return inputs.leftPosition <= ElevatorConstants.rotationsBeforeZero;
  }

  public Command driveWithSlowVoltageDown() {
    return run(() -> driveWithVoltage(Volts.of(ElevatorConstants.slowVoltageDown)));
  }

  public Command stop() {
    return run(() -> elevatorIO.runVoltage(Volts.of(0)));
  }

  public boolean isAtExtrema() {
    double signal = inputs.leftVelocity;
    return Math.abs(signal) < 0.2 || !inputs.lowerLimitSwitchState;
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    elevator.setLength(getElevatorHeight().in(Meters));

    noElevatorAlert.set(inputs.leftMotorAlive && inputs.rightMotorAlive);

    if (inputs.upperLimitSwitchState || inputs.lowerLimitSwitchState) {
      driveWithVoltage(Volts.of(0));
    }
  }
}
