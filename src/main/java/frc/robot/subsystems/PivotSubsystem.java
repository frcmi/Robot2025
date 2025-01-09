package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;

public class PivotSubsystem extends SubsystemBase {
    TalonFX pivotMotor = new TalonFX(0); //TODO: the id is not zero chat
    PositionTorqueCurrentFOC motorPositionControl = new PositionTorqueCurrentFOC(Angle.ofBaseUnits(2, Units.Degrees)).withSlot(0);
    Slot0Configs slot0Configs = new Slot0Configs() //TODO: pls tune
    .withKP(1.0)
    .withKI(0.0)
    .withKD(0.0)
    .withKS(0.0)
    .withKV(0.0)
    .withKA(0.0)
    .withKG(0.0); //Gravity keeps us on the ground...

    public PivotSubsystem() {
        setDefaultCommand(null);
        pivotMotor.getConfigurator().apply(slot0Configs);
    }
    public Command setAngle(Angle angle) {
        return runOnce(() -> {
            pivotMotor.setControl(motorPositionControl.withPosition(angle));
        });
    }
    public Command goToIntakePosition() {
        return setAngle(Angle.ofBaseUnits(2, Units.Degrees)); //TODO: find what angles these are
    }
    public Command goToOutakePosition() {
        return setAngle(Angle.ofBaseUnits(2, Units.Degrees)); //TODO: find what angles these are
    }
}
