package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.BotType;
import frc.robot.Constants.PivotConstants;
import frc.lib.ultralogger.UltraDoubleLog;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;

public class PivotSubsystem extends SubsystemBase {
    Alert limitPassedAlert = new Alert("Pivot motor limit has been exceeded! FIX IT!", AlertType.kError);
    TalonFXS pivotMotorAlpha;// = new TalonFXS(PivotConstants.motorID);
    TalonFX pivotMotorTurbo;
    StatusSignal<AngularVelocity> velocity; // = pivotMotor.getVelocity();
    Alert nopivotAlert = new Alert("Pivot motor not detected!", AlertType.kError);
    // private final PositionTorqueCurrentFOC motorPositionControl = new PositionTorqueCurrentFOC(Degrees.of(0));
    private final VoltageOut motorVoltageControl = new VoltageOut(Volts.of(0));//.withEnableFOC(true);

    MechanismLigament2d pivotLigament2d;
    // TalonFXSSimState talonFXSim = pivotMotor.getSimState();
    private static final double kGearRatio = 10.0;
    private final DCMotorSim m_motorSimModel = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      DCMotor.getKrakenX60Foc(1), 0.001, kGearRatio
    ),
      DCMotor.getKrakenX60Foc(1)
    );

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    private final UltraDoubleLog setpointPublisher = new UltraDoubleLog("Pivot/Setpoint Rotations");
    private final UltraDoubleLog pidPublisher = new UltraDoubleLog("Pivot/PID Output");
    private final UltraDoubleLog ffPublisher = new UltraDoubleLog("Pivot/FF Output");
    private final UltraDoubleLog anglePublisher = new UltraDoubleLog("Pivot/Angle");

    // private final UltraDoubleLog encoderPublisher = new UltraDoubleLog("Pivot/Encoder Rotations");
    // private final UltraDoubleLog velPublisher = new UltraDoubleLog("Pivot/Velocity");
    private ProfiledPIDController pid = new ProfiledPIDController(PivotConstants.TurboBot.kP, PivotConstants.TurboBot.kI, PivotConstants.TurboBot.kD, new Constraints(PivotConstants.maxVelocity, PivotConstants.maxAccel));
    private ArmFeedforward feedforward = new ArmFeedforward(PivotConstants.TurboBot.kS, PivotConstants.TurboBot.kG, 0, 0);

    public final SysIdRoutine pivotSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.75).per(Seconds),
            Volts.of(1.5),
            Seconds.of(5),
            (state) -> SignalLogger.writeString("SysIdPivot_State", state.toString())
        ), 
        new Mechanism(this::driveWithVoltage, null, this),
        this
    );

    private double offset = PivotConstants.TurboBot.offset;
    private double discontinuityPoint = PivotConstants.TurboBot.discontinuity;

    public PivotSubsystem(BotType bot, MechanismLigament2d elevatorLigament) {
        if (bot == BotType.ALPHA_BOT) {
            TalonFXSConfiguration configuration = new TalonFXSConfiguration();
            pivotMotorAlpha = new TalonFXS(PivotConstants.motorID);
            feedforward = new ArmFeedforward(PivotConstants.AlphaBot.kS, PivotConstants.AlphaBot.kG, 0, 0);
            offset = PivotConstants.AlphaBot.offset;
            discontinuityPoint = PivotConstants.AlphaBot.discontinuity;

            pid.setP(PivotConstants.AlphaBot.kP);
            pid.setI(PivotConstants.AlphaBot.kI);
            pid.setD(PivotConstants.AlphaBot.kD);

            configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            configuration.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
            configuration.Commutation.BrushedMotorWiring = BrushedMotorWiringValue.Leads_A_and_B;
            pivotMotorAlpha.getConfigurator().apply(configuration);
            pivotMotorAlpha.setNeutralMode(NeutralModeValue.Brake);
        } else {
            pivotMotorTurbo = new TalonFX(PivotConstants.motorID);
            pivotMotorTurbo.setNeutralMode(NeutralModeValue.Brake);
        }

        pid.setTolerance(Degrees.of(3.5).in(Radians));
        velocity.setUpdateFrequency(1000);
        
        pivotLigament2d = elevatorLigament.append(new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));

        // makes closeEnough return false on first poll after bot enabled
        pid.setGoal(PivotConstants.stowAngle.in(Radians));
        pid.calculate(0);

        setDefaultCommand(this.holdAngle());
    }

    public CommonTalon getPivotMotor() {
        if (pivotMotorTurbo != null) {
            return pivotMotorTurbo;
        }

        return pivotMotorAlpha;
    }

    public boolean isAlive() {
        if (pivotMotorTurbo != null) {
            return pivotMotorTurbo.isAlive();
        }

        return pivotMotorAlpha.isAlive();
    }

    public boolean closeEnough() {
        return Math.abs(pid.getPositionError()) < Degrees.of(2).in(Radians);
    }

    private Angle currentAngle = PivotConstants.stowAngle;

    public void setAngle(Angle angle) {
        currentAngle = angle;
    }

    public Command holdAngle() {
        return run(() -> {
            double setpoint = currentAngle.in(Radians);

            setpointPublisher.update(currentAngle.in(Rotations));
            double currentAngle = getEncoder().in(Radians);
            double voltage = pid.calculate(currentAngle, setpoint);
            pidPublisher.update(voltage);
            double signum = Math.signum(voltage);
            if (pid.atSetpoint() && pid.getPositionError() < 0) {
                voltage = 0;
                if (closeEnough()) {
                    signum = 0;
                }
            }
            double ff = feedforward.calculate(currentAngle, signum);
            ffPublisher.update(ff);
            if (DriverStation.isEnabled()) {
                getPivotMotor().setControl(motorVoltageControl.withOutput(Volts.of((voltage + ff))));
            }
        });
    }

    public void goToAngle(int level) {
        switch(level) {
            case 0:
                goToFloorAngle();
            case 1:
                goToOnCoralAngle();
            case 2:
                goToReefOneAngle();
            case 3:
                goToReefTwoAngle();
            case 4:
                goToBargeAngle();
            default:
                goToFloorAngle();
        }
    }

    public void goToFloorAngle() {
        setAngle(PivotConstants.floorAngle);
    }
    public void goToOnCoralAngle() {
        setAngle(PivotConstants.onCoralAngle);
    }
    public void goToReefOneAngle() {
        setAngle(PivotConstants.reefOneAngle);
    }
    public void goToReefTwoAngle() {
        setAngle(PivotConstants.reefTwoAngle);
    }
    public void goToBargeAngle() {
        setAngle(PivotConstants.bargeAngle);
    }

    private void driveWithVoltage(Voltage volts) {
        getPivotMotor().setControl(motorVoltageControl.withOutput(volts));
    }

    public Angle getEncoder() {
        double encoderValue = encoder.get() + offset;
        if (encoderValue <= discontinuityPoint) {
            encoderValue += 1;
        }

        return Rotations.of(encoderValue);
    }

    public void sysIDLog() {
        double encoderValue = getEncoder().in(Rotations);

        SignalLogger.writeDouble("Pivot Angle", encoderValue);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Arm close enough", closeEnough());
        anglePublisher.update(getEncoder().in(Rotations));
        if(getPivotMotor().getPosition().getValueAsDouble() <= PivotConstants.minAngle.in(Rotations) &&
            getPivotMotor().getPosition().getValueAsDouble() >= PivotConstants.maxAngle.in(Rotations)) {
            getPivotMotor().setControl(new NeutralOut());
           limitPassedAlert.set(true);
        }
        nopivotAlert.set(!isAlive());
        if (Robot.isReal()) {
            pivotLigament2d.setAngle(getPivotMotor().getPosition().getValue().in(Degrees));
        } else if (Robot.isSimulation()) {
            pivotLigament2d.setAngle(getPivotMotor().getPosition().getValue().minus(Degrees.of(90)).in(Degrees));
        }
    }
    @Override
    public void simulationPeriodic() {
        if (pivotMotorTurbo != null) {
            var talonFXSim = pivotMotorTurbo.getSimState();
            talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
            var motorVoltage = talonFXSim.getMotorVoltageMeasure();
            m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
            m_motorSimModel.update(0.020);
            talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
            talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
            return;
        } else {
            var talonFXSSim = pivotMotorAlpha.getSimState();
            talonFXSSim.setSupplyVoltage(RobotController.getBatteryVoltage());
            var motorVoltage = talonFXSSim.getMotorVoltageMeasure();
            m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
            m_motorSimModel.update(0.020);
            talonFXSSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
            talonFXSSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
        }
    }
}
