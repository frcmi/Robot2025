package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;

import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {
    private double setHeight = ElevatorConstants.minElevatorHeight;
    // Left is main, right is follower
    private final TalonFX elevatorMotorLeft = new TalonFX(9);
    private final TalonFX elevatorMotorRight = new TalonFX(10);
    // 10
    private final TalonFXSimState simState = elevatorMotorLeft.getSimState();
    Alert noelevAlert = new Alert("Elevator motor not detected!", AlertType.kError);

    SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(1).per(Seconds),
            Volts.of(3),
            Seconds.of(10),
            (state) -> SignalLogger.writeString("state", state.toString())
        ), 
        new Mechanism(this::driveWithVoltage, null, this)
    );

    private final Slot0Configs configs = new Slot0Configs()
        .withKP(ElevatorConstants.kP)
        .withKI(ElevatorConstants.kI)
        .withKD(ElevatorConstants.kD)
        .withKS(ElevatorConstants.kS)
        .withKV(ElevatorConstants.kV)
        .withKA(ElevatorConstants.kA)
        .withKG(ElevatorConstants.kG)
        .withGravityType(GravityTypeValue.Elevator_Static);
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

    private final PositionTorqueCurrentFOC elevatorPositionControl = new PositionTorqueCurrentFOC(Degrees.of(0)).withSlot(0);
    private final VoltageOut elevatorVoltageControl = new VoltageOut(0).withEnableFOC(true);

    // there will be at least one limit switch and an encoder to track the position of the elevator
    public ElevatorSubsystem() {
        elevatorMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotorRight.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotorLeft.getConfigurator().apply(configs);
        elevatorMotorRight.getConfigurator().apply(configs);
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
        return sysIdRoutine.quasistatic(dir);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return sysIdRoutine.dynamic(dir);
    }

    /** Height is relative to bottom of motor
     */
    public Distance getElevatorHeight() {
        return Meters.of(elevatorMotorLeft.getPosition().getValueAsDouble() * ElevatorConstants.rotationsPerMeter);
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            elevator.setLength(getElevatorHeight().in(Meters));
        }
        
        noelevAlert.set(!elevatorMotorLeft.isAlive());
        
        // TODO: brandon says he'll fix this
        // if (limitSwitch.get()) {
        //     extendingMotor.setPosition(0);
        // }
    }

    private double rotorPosition = 0;

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
