package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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
    // this is the motor that will extend the elevator
    private final TalonFX extendingMotor = new TalonFX(0);

    private final DCMotorSim simMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500Foc(1), ElevatorConstants.elevatorInertia, ElevatorConstants.gearRatio), DCMotor.getFalcon500Foc(0));

    private final Slot0Configs configs = new Slot0Configs()
        .withKP(ElevatorConstants.kP)
        .withKI(ElevatorConstants.kI)
        .withKD(ElevatorConstants.kD)
        .withKS(ElevatorConstants.kS)
        .withKV(ElevatorConstants.kV)
        .withKA(ElevatorConstants.kA)
        .withKG(ElevatorConstants.kG);
    // will track the position of the elevator
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ElevatorConstants.absoluteEncoderChannel);
    
    // will figure out dimensions later
    private final Mechanism2d windmill = new Mechanism2d(0.508, 2.4384);

    private final MechanismRoot2d root = windmill.getRoot("elevator", 0.737/2, 0.737);
    public final MechanismLigament2d elevator = root.append(new MechanismLigament2d("elevator", 2, 90, 10, new Color8Bit(255, 0, 0)));
    
    // limit switch for when the elevator is not extended
    private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.limitSwitchChannel);

    PositionTorqueCurrentFOC elevatorPositionControl = new PositionTorqueCurrentFOC(Degrees.of(0)).withSlot(0);

    // there will be at least one limit switch and an encoder to track the position of the elevator
    public ElevatorSubsystem() {
        extendingMotor.setNeutralMode(NeutralModeValue.Brake);
        extendingMotor.getConfigurator().apply(configs);
        extendingMotor.setPosition(encoder.get() + ElevatorConstants.absoluteEncoderOffset.in(Rotations));
        // extendingMotor.setControl(new DutyCycleOut(0.5));

    }
    // rev throughbore encoder, limit on bottom

    // I would assume that there is only going to be one motor to extend the elevator but we will see
    public Command extendArm(double rotations){
        return runOnce(() -> {
            extendingMotor.setControl(elevatorPositionControl.withPosition(rotations));
            
        });
    }

    public Command goToFloorHeightCommand() {
        return (extendArm(ElevatorConstants.floorHeight/ElevatorConstants.inchesPerRotation));
    }

    public Command goToOnCoralHeightCommand() {
        return (extendArm(ElevatorConstants.onCoralHeight/ElevatorConstants.inchesPerRotation));
    }

    public Command goToReefOneHeightCommand() {
        return (extendArm(ElevatorConstants.reefOneHeight/ElevatorConstants.inchesPerRotation));
    }

    public Command goToReefTwoHeightCommand() {
        return (extendArm(ElevatorConstants.reefTwoHeight/ElevatorConstants.inchesPerRotation));
    }

    public Command goToBargeHeightCommand() {
        return (extendArm(ElevatorConstants.bargeHeight/ElevatorConstants.inchesPerRotation));
    }

    /** Height is relative to bottom of motor
     */
    public Distance getElevatorHeight() {
        return Meters.of(extendingMotor.getPosition().getValueAsDouble() * ElevatorConstants.metersPerRotation);
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            elevator.setLength(getElevatorHeight().in(Meters));
        }
        if (limitSwitch.get()) {
            extendingMotor.setPosition(0);
        }
        SmartDashboard.putData("Windmill", windmill);
    }

    @Override
    public void simulationPeriodic() {
        extendingMotor.setControl(new DutyCycleOut(0.5));
        

        TalonFXSimState simState = extendingMotor.getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        simMotor.setInputVoltage(simState.getMotorVoltage());
        simMotor.update(0.020);
        simState.setRawRotorPosition(ElevatorConstants.gearRatio * simMotor.getAngularPositionRotations());
        simState.setRotorVelocity(ElevatorConstants.gearRatio * Units.radiansToRotations(simMotor.getAngularVelocityRadPerSec()));

        elevator.setLength(ElevatorConstants.gearRatio * simMotor.getAngularPositionRotations() * ElevatorConstants.metersPerRotation);
    }

    
    
    // the speed of the motor will depend on how it is set up and how much power is needed
    // public Command lowerArm(double speed){
    //     return run(() -> {
    //      if(limitSwitch.get()){
    //         extendingMotor.set(0.0);
    //      } else{
    //         extendingMotor.set(-speed);
    //      }  
    //     });
    // }
}
