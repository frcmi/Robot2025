package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          ElevatorConstants.gearRatio,
          ElevatorConstants.elevatorWeight,
          ElevatorConstants.drumRadius,
          ElevatorConstants.minElevatorHeight,
          ElevatorConstants.maxElevatorHeight,
          true,
          0.1);

  private final PIDController pid =
      new PIDController(
          ElevatorConstants.simBotConfigs.kP,
          ElevatorConstants.simBotConfigs.kI,
          ElevatorConstants.simBotConfigs.kD);

  private Voltage setVolts = null;

  private Angle setPosition = null;

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    if (setVolts != null) {
      elevatorSim.setInputVoltage(setVolts.in(Volts));
    } else if (setPosition != null) {
      elevatorSim.setInputVoltage(pid.calculate(inputs.leftPosition, setPosition.in(Rotations)));
    }

    elevatorSim.update(0.020);

    inputs.leftPosition =
        (elevatorSim.getPositionMeters() - ElevatorConstants.minElevatorHeight)
            * ElevatorConstants.rotationsPerMeter;
    inputs.leftVelocity =
        elevatorSim.getVelocityMetersPerSecond() * ElevatorConstants.rotationsPerMeter;
    inputs.leftSetPoint = setPosition.in(Rotations);
    inputs.leftMotorAlive = true;
    inputs.leftTemperature = 0;

    if (inputs.leftPosition < ElevatorConstants.minElevatorHeight
        || inputs.leftPosition > ElevatorConstants.maxElevatorHeight) {
      inputs.leftStatorCurrent = 200;
    } else {
      inputs.leftStatorCurrent = 0;
    }

    inputs.rightPosition = -inputs.leftPosition;
    inputs.rightMotorAlive = true;
    inputs.rightTemperature = 0;
  }

  @Override
  public void runVoltage(Voltage volts) {
    setVolts = volts;
    setPosition = null;
  }

  @Override
  public void runPositionControl(Angle position) {
    setPosition = position;
    setVolts = null;
  }
}
