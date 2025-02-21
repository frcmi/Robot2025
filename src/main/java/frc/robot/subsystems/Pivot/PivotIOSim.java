package frc.robot.subsystems.Pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PivotConstants;

public class PivotIOSim implements PivotIO {
  private final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          PivotConstants.gearRatio,
          PivotConstants.momentOfInertia,
          PivotConstants.length,
          PivotConstants.minAngle.in(Radians),
          PivotConstants.maxAngle.in(Radians),
          true,
          0);

  private Voltage setVolts = null;

  public PivotIOSim() {
    pivotSim.setState(Math.PI / 2 - 0.01, 0);
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    if (setVolts != null) {
      SmartDashboard.putNumber("volty wolties", setVolts.in(Volts));
      pivotSim.setInputVoltage(setVolts.in(Volts));
    }

    pivotSim.update(0.020);

    inputs.position = Radians.of(pivotSim.getAngleRads()).in(Rotations);
    inputs.encoderPosition = inputs.position;
    inputs.velocity = Radians.of(pivotSim.getVelocityRadPerSec()).in(Rotations);

    inputs.motorAlive = true;
    inputs.temperature = 0;

    inputs.statorCurrent = 0;
  }

  @Override
  public void runVoltage(Voltage volts) {
    setVolts = volts;
  }
}
