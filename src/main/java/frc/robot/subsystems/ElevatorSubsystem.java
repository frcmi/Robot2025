package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.lib.ultralogger.UltraDoubleLog;
import frc.robot.Constants.BotType;
import frc.robot.Constants.ElevatorConstants;

import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {
    // Left is main, right is follower
    private final TalonFX elevatorMotorLeft = new TalonFX(9);
    private final TalonFX elevatorMotorRight = new TalonFX(10);
    // 10
    private final TalonFXSimState simState = elevatorMotorLeft.getSimState();
    Alert noelevAlert = new Alert("Elevator motor not detected!", AlertType.kError);

    public final SysIdRoutine elevatorSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(1).per(Seconds),
            Volts.of(3),
            Seconds.of(10),
            (state) -> SignalLogger.writeString("state", state.toString())
        ), 
        new Mechanism(this::driveWithVoltage, null, this)
    );

    // will track the position of the elevator
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ElevatorConstants.absoluteEncoderChannel);
    
    // will figure out dimensions later
    private final Mechanism2d windmill = new Mechanism2d(1.5, 2.4384);

    private final MechanismRoot2d root = windmill.getRoot("elevator", 0.75, 0);
    public final MechanismLigament2d elevator = root.append(new MechanismLigament2d("elevator", 2, 90, 10, new Color8Bit(255, 0, 0)));
    
    // limit switch for when the elevator is not extended
    private final ElevatorSim elevatorSim = new ElevatorSim(
        DCMotor.getKrakenX60Foc(1), 
        ElevatorConstants.gearRatio,
        ElevatorConstants.elevatorWeight,
        ElevatorConstants.drumRadius,
        ElevatorConstants.minElevatorHeight,
        ElevatorConstants.maxElevatorHeight,
        true, 
        0.1);

    private final PositionTorqueCurrentFOC elevatorPositionControl = new PositionTorqueCurrentFOC(Degrees.of(0));
    private final VoltageOut elevatorVoltageControl = new VoltageOut(0).withEnableFOC(true);

    private final DigitalInput magneticLimitSwitch = new DigitalInput(ElevatorConstants.magneticLimitSwitchID);

    // there will be at least one limit switch and an encoder to track the position of the elevator
    public ElevatorSubsystem(BotType bot) {
        elevatorPositionControl.withSlot(bot.slotId);

        elevatorMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotorRight.setNeutralMode(NeutralModeValue.Brake);

        elevatorMotorLeft.getConfigurator().apply(ElevatorConstants.realBotConfigs);
        elevatorMotorLeft.getConfigurator().apply(ElevatorConstants.alphaBotConfigs);
        elevatorMotorRight.getConfigurator().apply(ElevatorConstants.realBotConfigs);
        elevatorMotorRight.getConfigurator().apply(ElevatorConstants.alphaBotConfigs);

        elevatorMotorLeft.setPosition(encoder.get() + ElevatorConstants.absoluteEncoderOffset.in(Rotations));
        elevatorMotorRight.setPosition(encoder.get() + ElevatorConstants.absoluteEncoderOffset.in(Rotations));
        elevatorMotorRight.setControl(new Follower(9, true));

        simState.Orientation = ChassisReference.CounterClockwise_Positive;
        SmartDashboard.putData("Windmill", windmill);
    }
    // rev throughbore encoder, limit on bottom

    // I would assume that there is only going to be one motor to extend the elevator but we will see
    public Command extendArm(double rotations){
        return runOnce(() -> {
            elevatorMotorLeft.setControl(elevatorPositionControl.withPosition(rotations));
        });
    }

    public void driveWithVoltage(Voltage volts) {
        elevatorMotorLeft.setControl(elevatorVoltageControl.withOutput(volts));
    }

    public Command goToHeight(int level) {
        switch(level) {
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

    private Command goToFloorHeightCommand() {
        return (extendArm(ElevatorConstants.floorHeight * ElevatorConstants.rotationsPerMeter));
    }

    private Command goToOnCoralHeightCommand() {
        return (extendArm(ElevatorConstants.onCoralHeight * ElevatorConstants.rotationsPerMeter));
    }

    private Command goToReefOneHeightCommand() {
        return (extendArm(ElevatorConstants.reefOneHeight * ElevatorConstants.rotationsPerMeter));
    }

    private Command goToReefTwoHeightCommand() {
        return (extendArm(ElevatorConstants.reefTwoHeight * ElevatorConstants.rotationsPerMeter));
    }

    private Command goToBargeHeightCommand() {
        return (extendArm(ElevatorConstants.bargeHeight * ElevatorConstants.rotationsPerMeter));
    }

    public Command sysIdQuazistatic(SysIdRoutine.Direction dir) {
        return elevatorSysIdRoutine.quasistatic(dir);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return elevatorSysIdRoutine.dynamic(dir);
    }

    /** Height is relative to bottom of motor
     */
    public Distance getElevatorHeight() {
        return Meters.of(elevatorMotorLeft.getPosition().getValueAsDouble() * ElevatorConstants.rotationsPerMeter);
    }

    public boolean isCurrentSpiked() {
        return elevatorMotorLeft.getStatorCurrent().getValueAsDouble() > 1.0;
    }

    public boolean isRotationsAlmostAtZero() {
        return elevatorMotorLeft.getPosition().getValueAsDouble() <= ElevatorConstants.rotationsBeforeZero;
    }

    public Command driveWithSlowVoltageDown() {
        return run(() -> driveWithVoltage(Volts.of(ElevatorConstants.slowVoltageDown)));
    }

    public Command stop() {
        return run(() -> elevatorMotorLeft.setControl(new VoltageOut(0).withLimitReverseMotion(true)));
    }

    public Command zeroElevatorDown() {
        return goToFloorHeightCommand().until(() -> isRotationsAlmostAtZero()).andThen(driveWithSlowVoltageDown())
        .until(() -> { return isCurrentSpiked() || magneticLimitSwitch.get(); }).andThen(stop());
    }

    public boolean isRotationsAlmostAtMax() {
        return elevatorMotorLeft.getPosition().getValueAsDouble() >= ElevatorConstants.rotationsBeforeMaxHeight;
    }

    public Command driveWithSlowVoltageUp() {
        return run(() -> driveWithVoltage(Volts.of(ElevatorConstants.slowVoltageUp)));
    }

    // this command will definently be changed due to how the elevator needs to be slowed down
    public Command zeroElevatorUp() {
        return goToBargeHeightCommand().until(() -> isRotationsAlmostAtMax()).andThen(driveWithSlowVoltageUp())
        .until(() -> { return isCurrentSpiked() || magneticLimitSwitch.get(); }).andThen(stop());
    }

    UltraDoubleLog setPose = new UltraDoubleLog("Elevator/Set Rotations");
    UltraDoubleLog currentPose = new UltraDoubleLog("Elevator/Current Rotations");
    StatusSignal<Double> setPoseSignal = elevatorMotorLeft.getClosedLoopReference();
    StatusSignal<Angle> currentPoseSignal = elevatorMotorLeft.getPosition();


    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(setPoseSignal, currentPoseSignal);
        setPose.update(setPoseSignal.getValue());
        currentPose.update(currentPoseSignal.getValueAsDouble());
        if (RobotBase.isReal()) {
            elevator.setLength(getElevatorHeight().in(Meters));
        }
        
        noelevAlert.set(!elevatorMotorLeft.isAlive());
        
        if (magneticLimitSwitch.get()) {
            driveWithVoltage(Volts.of(0));
            System.out.println("Elevator went past limit switch");
        }
    }

    @Override
    public void simulationPeriodic() {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        elevatorSim.setInputVoltage(simState.getMotorVoltage());
        elevatorSim.update(0.020);
        simState.setRawRotorPosition((elevatorSim.getPositionMeters() - ElevatorConstants.minElevatorHeight) * ElevatorConstants.rotationsPerMeter);
        simState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * ElevatorConstants.rotationsPerMeter);
        elevator.setLength(elevatorSim.getPositionMeters());
    }
}
