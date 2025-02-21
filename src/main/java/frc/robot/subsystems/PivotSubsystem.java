package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
    TalonFX pivotMotor = new TalonFX(PivotConstants.motorID);
    StatusSignal<AngularVelocity> velocity = pivotMotor.getVelocity();
    Alert nopivotAlert = new Alert("Pivot motor not detected!", AlertType.kError);
    // private final PositionTorqueCurrentFOC motorPositionControl = new PositionTorqueCurrentFOC(Degrees.of(0));
    private final VoltageOut motorVoltageControl = new VoltageOut(Volts.of(0));//.withEnableFOC(true);

    MechanismLigament2d pivotLigament2d;
    TalonFXSimState talonFXSim = pivotMotor.getSimState();
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
    private final UltraDoubleLog anglePublisher = new UltraDoubleLog("Pivot/Encoder Rotations");

    // private final UltraDoubleLog encoderPublisher = new UltraDoubleLog("Pivot/Encoder Rotations");
    // private final UltraDoubleLog velPublisher = new UltraDoubleLog("Pivot/Velocity");
    private ProfiledPIDController pid = new ProfiledPIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD, new Constraints(PivotConstants.maxVelocity, PivotConstants.maxAccel));
    private ArmFeedforward feedforward = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV, PivotConstants.kA);


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

    public PivotSubsystem(BotType bot, MechanismLigament2d elevatorLigament) {
        pid.setTolerance(Degrees.of(3.5).in(Radians));
        velocity.setUpdateFrequency(1000);

        pivotLigament2d = elevatorLigament.append(new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotMotor.getConfigurator().apply(configuration);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        Angle stow = Rotations.of(0.27261940556548514);

        // makes closeEnough return false on first poll after bot enabled
        pid.setGoal(stow.in(Radians));
        pid.calculate(0);


        this.setAngle(stow).ignoringDisable(true).schedule();
    }

    public boolean closeEnough() {
        return Math.abs(pid.getPositionError()) < Degrees.of(2).in(Radians);
    }

    public Command holdPosition() {
        return run(() -> {
            double ff = feedforward.calculate(getEncoder().in(Radians), 0);

            pidPublisher.update(0.0);
            ffPublisher.update(ff);

            pivotMotor.setControl(motorVoltageControl.withOutput(Volts.of(ff)));
        });
    }

    public Command setAngle(Angle angle) {
        double setpoint = angle.in(Radians);
        return run(() -> {
            setpointPublisher.update(angle.in(Rotations));
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
                pivotMotor.setControl(motorVoltageControl.withOutput(Volts.of((voltage + ff))));
            }
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

    public Angle getEncoder() {
        double encoderValue = encoder.get() - 0.75 - 0.38853565346339136 +0.05;
        if (encoderValue <= -0.3) {
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
        if(pivotMotor.getPosition().getValueAsDouble() <= PivotConstants.minAngle.in(Rotations) &&
           pivotMotor.getPosition().getValueAsDouble() >= PivotConstants.maxAngle.in(Rotations)) {
           pivotMotor.stopMotor();
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
