package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.BotType;
import frc.robot.Constants.PivotConstants;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;

public class PivotSubsystem extends SubsystemBase {
    Alert limitPassedAlert = new Alert("Pivot motor limit has been exceeded! FIX IT!", AlertType.kError);
    TalonFX pivotMotor = new TalonFX(PivotConstants.motorID);
    Alert nopivotAlert = new Alert("Pivot motor not detected!", AlertType.kError);
    private final PositionTorqueCurrentFOC motorPositionControl = new PositionTorqueCurrentFOC(Degrees.of(0));
    private final VoltageOut motorVoltageControl = new VoltageOut(Volts.of(0)).withEnableFOC(true);
    Slot0Configs slot0Configs = new Slot0Configs() //TODO: pls tune
        .withKP(PivotConstants.kP)
        .withKI(PivotConstants.kI)
        .withKD(PivotConstants.kD)
        .withKS(PivotConstants.kS)
        .withKV(PivotConstants.kV)
        .withKA(PivotConstants.kA)
        .withKG(PivotConstants.kG); // Gravity keeps us on the ground...
    MechanismLigament2d pivotLigament2d;
    TalonFXSimState talonFXSim = pivotMotor.getSimState();
    private static final double kGearRatio = 10.0;
    private final DCMotorSim m_motorSimModel = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      DCMotor.getKrakenX60Foc(1), 0.001, kGearRatio
    ),
      DCMotor.getKrakenX60Foc(1)
    );

    public final SysIdRoutine pivotSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(1).per(Seconds),
            Volts.of(3),
            Seconds.of(10),
            (state) -> SignalLogger.writeString("state", state.toString())
        ), 
        new Mechanism(this::driveWithVoltage, null, this)
    );

    public PivotSubsystem(BotType bot, MechanismLigament2d elevatorLigament) {
        motorPositionControl.withSlot(bot.slotId);

        pivotMotor.getConfigurator().apply(slot0Configs);
        pivotLigament2d = elevatorLigament.append(new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    public Command setAngle(Angle angle) {
        return runOnce(() -> {
            pivotMotor.setControl(motorPositionControl.withPosition(angle));
        });
    }
    public Command goToAngle(int level) {
        switch(level) {
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

    private void driveWithVoltage(Voltage volts) {
        pivotMotor.setControl(motorVoltageControl.withOutput(volts));
    }

    @Override
    public void periodic() {
        if(pivotMotor.getPosition().getValueAsDouble() <= PivotConstants.minAngle.in(Rotations) &&
           pivotMotor.getPosition().getValueAsDouble() >= PivotConstants.maxAngle.in(Rotations)) {
        //    pivotMotor.stopMotor();
           limitPassedAlert.set(true);
        }
        nopivotAlert.set(!pivotMotor.isAlive());
        if (Robot.isReal()) {
            pivotLigament2d.setAngle(pivotMotor.getPosition().getValue().in(Degrees));
        } else if (Robot.isSimulation()) {
            pivotLigament2d.setAngle(pivotMotor.getPosition().getValue().minus(Degrees.of(90)).in(Degrees));
        }
    }
    @Override
    public void simulationPeriodic() {
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();
        m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        m_motorSimModel.update(0.020);
        talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
        talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
    }
}
