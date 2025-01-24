package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;

import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {
    private double setHeight = ElevatorConstants.minElevatorHeight;
    // this is the motor that will extend the elevator
    private final TalonFX elevatorMotor = new TalonFX(20);
    private final TalonFXSimState simState = elevatorMotor.getSimState();
    Alert noelevAlert = new Alert("Elevator motor not detected!", AlertType.kError);

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

    // there will be at least one limit switch and an encoder to track the position of the elevator
    public ElevatorSubsystem() {
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor.getConfigurator().apply(configs);
        elevatorMotor.setPosition(encoder.get() + ElevatorConstants.absoluteEncoderOffset.in(Rotations));
        simState.Orientation = ChassisReference.CounterClockwise_Positive;
        SmartDashboard.putData("Windmill", windmill);
    }
    // rev throughbore encoder, limit on bottom

    // I would assume that there is only going to be one motor to extend the elevator but we will see
    public Command extendArm(double rotations){
        return runOnce(() -> {
            elevatorMotor.setControl(elevatorPositionControl.withPosition(rotations));
        });
    }

    public Command goToFloorHeightCommand() {
        return (extendArm(ElevatorConstants.floorHeight * ElevatorConstants.rotationsPerMeter));
    }

    public Command goToOnCoralHeightCommand() {
        return (extendArm(ElevatorConstants.onCoralHeight * ElevatorConstants.rotationsPerMeter));
    }

    public Command goToReefOneHeightCommand() {
        return (extendArm(ElevatorConstants.reefOneHeight * ElevatorConstants.rotationsPerMeter));
    }

    public Command goToReefTwoHeightCommand() {
        return (extendArm(ElevatorConstants.reefTwoHeight * ElevatorConstants.rotationsPerMeter));
    }

    public Command goToBargeHeightCommand() {
        return (extendArm(ElevatorConstants.bargeHeight * ElevatorConstants.rotationsPerMeter));
    }

    /** Height is relative to bottom of motor
     */
    public Distance getElevatorHeight() {
        return Meters.of(elevatorMotor.getPosition().getValueAsDouble() * ElevatorConstants.rotationsPerMeter);
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            elevator.setLength(getElevatorHeight().in(Meters));
        }
        
        noelevAlert.set(!elevatorMotor.isAlive());
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
