package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.ElevatorConstants;

import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    // private final TalonFXSimState simState = elevatorMotorLeft.getSimState();
    
    private final Alert noelevAlert = new Alert("Elevator motor not detected!", AlertType.kError);

    public final SysIdRoutine elevatorSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(1).per(Seconds),
            Volts.of(3),
            Seconds.of(10),
            (state) -> SignalLogger.writeString("state", state.toString())
        ), 
        new Mechanism(this::driveWithVoltage, null, this)
    );

    // private final Mechanism2d windmill = new Mechanism2d(1.5, 2.4384);

    // private final MechanismRoot2d root = windmill.getRoot("elevator", 0.75, 0);
    // public final MechanismLigament2d elevator = root.append(new MechanismLigament2d("elevator", 2, 90, 10, new Color8Bit(255, 0, 0)));
    
    // private final ElevatorSim elevatorSim = new ElevatorSim(
    //     DCMotor.getKrakenX60Foc(1), 
    //     ElevatorConstants.gearRatio,
    //     ElevatorConstants.elevatorWeight,
    //     ElevatorConstants.drumRadius,
    //     ElevatorConstants.minElevatorHeight,
    //     ElevatorConstants.maxElevatorHeight,
    //     true, 
    //     0.1);

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
        // simState.Orientation = ChassisReference.CounterClockwise_Positive;
        // SmartDashboard.putData("Windmill", windmill);
    }

    public Command extendArm(double rotations){
        return run(() -> {});
        // return runOnce(() -> {
        //     elevatorMotorLeft.setControl(elevatorPositionControl.withPosition(rotations));
        // });
    }

    public void driveWithVoltage(Voltage volts) {
        elevatorIO.runVoltage(volts);
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

    public Command sysIdQuazistatic(SysIdRoutine.Direction dir) {
        return elevatorSysIdRoutine.quasistatic(dir);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return elevatorSysIdRoutine.dynamic(dir);
    }

    /** Height is relative to bottom of motor
     */
    public Distance getElevatorHeight() {
        return Meters.of(inputs.leftPosition * ElevatorConstants.rotationsPerMeter);
    }

    public boolean isCurrentSpiked() {
        return inputs.leftStatorCurrent > 1.0;
    }

    public boolean isRotationsAlmostAtZero() {
        return inputs.leftPosition <= ElevatorConstants.rotationsBeforeZero;
    }

    public Command driveWithSlowVoltageDown() {
        return run(() -> driveWithVoltage(Volts.of(ElevatorConstants.slowVoltageDown)));
    }

    public Command stop() {
        return run(() -> elevatorIO.runVoltage(Volts.of(0)));
    }

    public Command zeroElevatorDown() {
        return goToFloorHeightCommand().until(() -> isRotationsAlmostAtZero()).andThen(driveWithSlowVoltageDown())
        .until(() -> { return isCurrentSpiked() || inputs.limitSwitchState; }).andThen(stop());
    }

    public boolean isRotationsAlmostAtMax() {
        return inputs.leftPosition >= ElevatorConstants.rotationsBeforeMaxHeight;
    }

    public Command driveWithSlowVoltageUp() {
        return run(() -> driveWithVoltage(Volts.of(ElevatorConstants.slowVoltageUp)));
    }

    public Command zeroElevatorUp() {
        return goToBargeHeightCommand().until(() -> isRotationsAlmostAtMax()).andThen(driveWithSlowVoltageUp())
        .until(() -> { return isCurrentSpiked() || inputs.limitSwitchState; }).andThen(stop());
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(inputs);
        // if (RobotBase.isReal()) {
        //     elevator.setLength(getElevatorHeight().in(Meters));
        // }
        
        noelevAlert.set(inputs.leftMotorAlive && inputs.rightMotorAlive);
        
        if (inputs.limitSwitchState) {
            driveWithVoltage(Volts.of(0));
        }
    }

    // TODO: implement a sim implementation of the elevator IO
    // @Override
    // public void simulationPeriodic() {
    //     simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    //     elevatorSim.setInputVoltage(simState.getMotorVoltage());
    //     elevatorSim.update(0.020);
    //     simState.setRawRotorPosition((elevatorSim.getPositionMeters() - ElevatorConstants.minElevatorHeight) * ElevatorConstants.rotationsPerMeter);
    //     simState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * ElevatorConstants.rotationsPerMeter);
    //     elevator.setLength(elevatorSim.getPositionMeters());
    // }
}
