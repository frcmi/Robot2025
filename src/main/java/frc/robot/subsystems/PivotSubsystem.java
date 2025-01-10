package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import static edu.wpi.first.units.Units.*;

public class PivotSubsystem extends SubsystemBase {
    Alert limitPassedAlert = new Alert("Pivot motor limit has been exceeded! FIX IT!", AlertType.kError);
    TalonFX pivotMotor = new TalonFX(PivotConstants.motorID);
    PositionTorqueCurrentFOC motorPositionControl = new PositionTorqueCurrentFOC(Degrees.of(0)).withSlot(0);
    Slot0Configs slot0Configs = new Slot0Configs() //TODO: pls tune
    .withKP(PivotConstants.kP)
    .withKI(PivotConstants.kI)
    .withKD(PivotConstants.kD)
    .withKS(PivotConstants.kS)
    .withKV(PivotConstants.kV)
    .withKA(PivotConstants.kA)
    .withKG(PivotConstants.kG); //Gravity keeps us on the ground...

    public PivotSubsystem() {
        setDefaultCommand(null);
        pivotMotor.getConfigurator().apply(slot0Configs);
    }
    public Command setAngle(Angle angle) {
        return runOnce(() -> {
            pivotMotor.setControl(motorPositionControl.withPosition(angle));
        });
    }
    public Command goToFloorPosition() {
        return setAngle(PivotConstants.floorAngle);
    }
    public Command goToOnCoralPosition() {
        return setAngle(PivotConstants.onCoralAngle);
    }
    public Command goToReefPosition() {
        return setAngle(PivotConstants.reefAngle);
    }
    public Command goToBargePosition() {
        return setAngle(PivotConstants.bargeAngle);
    }

    @Override
    public void periodic() {
        if(pivotMotor.getPosition().getValueAsDouble() <= PivotConstants.minAngle.in(Rotations) &&
           pivotMotor.getPosition().getValueAsDouble() >= PivotConstants.maxAngle.in(Rotations)) {
            pivotMotor.stopMotor();
            pivotMotor.setNeutralMode(NeutralModeValue.Brake);
            limitPassedAlert.set(true);
        }
    }
}
