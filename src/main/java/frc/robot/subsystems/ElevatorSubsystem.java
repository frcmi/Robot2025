package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
import frc.robot.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.lib.ultralogger.UltraDoubleLog;
import frc.lib.ultralogger.UltraStringLog;
import frc.lib.ultralogger.UltraSupplierLog;
import frc.robot.Constants.BotType;
import frc.robot.Constants.ElevatorConstants;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {
    // Left is main, right is follower
    private final TalonFX elevatorMotorLeft = new TalonFX(9);
    private final TalonFX followerMotor = new TalonFX(10);
    private final TalonFXSimState simState = elevatorMotorLeft.getSimState();
    Alert noelevAlert = new Alert("Elevator motor not detected!", AlertType.kError);
    
    private final DigitalInput upperDigitalInput = new DigitalInput(ElevatorConstants.upperLimitSwitchID);
    private final DigitalInput lowerDigitalInput = new DigitalInput(ElevatorConstants.lowerLimitSwitchID);

    public final SysIdRoutine elevatorSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(2).per(Seconds),
            Volts.of(3),
            Seconds.of(10),
            (state) -> SignalLogger.writeString("SysIdElevator_State", state.toString())
        ), 
        new Mechanism(this::driveWithVoltage, null, this),
        this,
        () -> !upperDigitalInput.get(),
        () -> !lowerDigitalInput.get()
    );

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

    private MotionMagicVoltage elevatorPositionControl = new MotionMagicVoltage(Degrees.of(0)).withEnableFOC(true);
    private final VoltageOut elevatorVoltageControl = new VoltageOut(0);

    private final UltraStringLog commandRunningPublisher = new UltraStringLog("Elevator/Command Running");
    private final UltraDoubleLog sysidVoltagePublisher = new UltraDoubleLog("Elevator/SysID Commanded Voltage");

    private final UltraSupplierLog leftVelocityPub = new UltraSupplierLog("Elevator/Left Velocity", elevatorMotorLeft.getVelocity());
    private final UltraSupplierLog leftTempPub = new UltraSupplierLog("Elevator/Left Temp", elevatorMotorLeft.getDeviceTemp());
    private final UltraSupplierLog leftPosePub = new UltraSupplierLog("Elevator/Left Pose", elevatorMotorLeft.getPosition());
    private final UltraSupplierLog rightVelocityPub = new UltraSupplierLog("Elevator/Right Velocity", followerMotor.getVelocity());
    private final UltraSupplierLog rightTempPub = new UltraSupplierLog("Elevator/Right Temp", followerMotor.getDeviceTemp());
    private final UltraSupplierLog rightPosePub = new UltraSupplierLog("Elevator/Right Pose", followerMotor.getPosition());
    private final Alert estopAlert = new Alert("Elevator E-Stopped", AlertType.kError);

    private boolean estop = false;

    // there will be at least one limit switch and an encoder to track the position of the elevator
    public ElevatorSubsystem(BotType bot) {
        elevatorPositionControl = elevatorPositionControl.withSlot(bot.slotId);

        SoftwareLimitSwitchConfigs softLimitConfig = new SoftwareLimitSwitchConfigs();
            // .withReverseSoftLimitThreshold(ElevatorConstants.absoluteBottom)
            // .withReverseSoftLimitEnable(true)
            // .withForwardSoftLimitThreshold(ElevatorConstants.absoluteTop)
            // .withForwardSoftLimitEnable(true);

        HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs()
            .withReverseLimitAutosetPositionValue(Rotation.of(0))
            .withReverseLimitAutosetPositionEnable(true)
            .withReverseLimitEnable(true);

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Rotations.per(Second).of(200))
            .withMotionMagicAcceleration(Rotations.per(Second).per(Second).of(350))
            .withMotionMagicJerk(Rotations.per(Second).per(Second).per(Second).of(500));

        elevatorMotorLeft.getConfigurator().apply(ElevatorConstants.realBotConfigs);
        elevatorMotorLeft.getConfigurator().apply(ElevatorConstants.alphaBotConfigs);
        elevatorMotorLeft.getConfigurator().apply(hardwareLimitSwitchConfigs);
        elevatorMotorLeft.getConfigurator().apply(softLimitConfig);
        elevatorMotorLeft.getConfigurator().apply(motionMagicConfigs);

        if (bot == BotType.ALPHA_BOT) {
            elevatorMotorLeft.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        } else {
            elevatorMotorLeft.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        }

        followerMotor.getConfigurator().apply(ElevatorConstants.realBotConfigs);
        followerMotor.getConfigurator().apply(ElevatorConstants.alphaBotConfigs);
        followerMotor.getConfigurator().apply(hardwareLimitSwitchConfigs);
        followerMotor.getConfigurator().apply(softLimitConfig);
        followerMotor.getConfigurator().apply(motionMagicConfigs);

        elevatorMotorLeft.setPosition(Rotations.of(1));
        followerMotor.setPosition(Rotations.of(1));

        elevatorMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);
        
        simState.Orientation = ChassisReference.CounterClockwise_Positive;
        SmartDashboard.putData("Windmill", windmill);

        setFollowerMode();
        setDefaultCommand(this.holdPose().withName("Default Hold Pose"));
        if (estop) {
            setDefaultCommand(stop());
        }
    }

    StatusSignal<Double> pidError = elevatorMotorLeft.getClosedLoopError();

    public boolean closeEnough() {
        StatusSignal.refreshAll(pidError);

        return Math.abs(pidError.getValueAsDouble()) < 0.1;

    }

    public void setFollowerMode() {
        if (estop) {
            elevatorMotorLeft.set(0);
            return;
        }
        
        followerMotor.setControl(new Follower(9, true));
    }

    // rev throughbore encoder, limit on bottom

    // I would assume that there is only going to be one motor to extend the elevator but we will see
    double poseToHold = ElevatorConstants.stowHeight;
    boolean pause = false;
    public Command holdPose(){
        // return run(() -> {});
        return run(() -> {
            // if (true) {
            //     return;
            // }

            if (estop) {
                elevatorMotorLeft.set(0);
                return;
            }

            SmartDashboard.putBoolean("Elevator pause", pause);
            if (pause) {
                return;
            }

            // if (poseToHold == ElevatorConstants.stowHeight) {
            //     StatusSignal.refreshAll(currentPoseSignal);
            //     if (Math.abs(currentPoseSignal.getValueAsDouble() - 0.2) <= 0.1) {
            //         pause = true;
            //         driveWithVoltage(Volts.of(0));
            //         return;
            //     }
            // }

            setFollowerMode();
            elevatorMotorLeft.setControl(elevatorPositionControl.withPosition(poseToHold));
        });
    }

    public void extendArm(double rotations) {
        SmartDashboard.putNumber("Elevator Setpoint", rotations);
        pause = false;
        poseToHold = rotations;
    }

    public void driveWithVoltage(Voltage volts) {
        if (estop) {
            elevatorMotorLeft.set(0);
            return;
        }

        sysidVoltagePublisher.update(volts.in(Volts));

        setFollowerMode();
        elevatorMotorLeft.setControl(elevatorVoltageControl.withOutput(volts));
    }

    public void goToHeight(int level) {
        switch(level) {
            case 0:
                zeroElevatorDown();
            case 1:
                goToOnCoralHeightCommand();
            case 2:
                goToReefOneHeightCommand();
            case 3:
                goToReefTwoHeightCommand();
            case 4:
                zeroElevatorUp();
            default:
                zeroElevatorDown();
        }
    }

    public void goToFloorHeightCommand() {
        extendArm(ElevatorConstants.floorHeight * ElevatorConstants.rotationsPerMeter);
    }

    public void goToOnCoralHeightCommand() {
        extendArm(ElevatorConstants.onCoralHeight * ElevatorConstants.rotationsPerMeter);
    }

    public void goToReefOneHeightCommand() {
        extendArm(ElevatorConstants.reefOneHeight * ElevatorConstants.rotationsPerMeter);
    }

    public void goToReefTwoHeightCommand() {
        extendArm(ElevatorConstants.reefTwoHeight * ElevatorConstants.rotationsPerMeter);
    }

    public void goToBargeHeightCommand() {
        extendArm(ElevatorConstants.bargeHeight * ElevatorConstants.rotationsPerMeter);
    }

    /** Height is relative to bottom of motor
     */
    public double getElevatorHeight() {
        return elevatorMotorLeft.getPosition().getValueAsDouble();
    }


    public boolean isAtExtrema() {
        var signal = elevatorMotorLeft.getVelocity();
        StatusSignal.refreshAll(signal);
        return Math.abs(signal.getValueAsDouble()) < 0.2 || !lowerDigitalInput.get();
    }

    public boolean isRotationsAlmostAtZero() {
        return elevatorMotorLeft.getPosition().getValueAsDouble() <= ElevatorConstants.rotationsBeforeZero;
    }

    public Command driveWithSlowVoltageDown() {
        return run(() -> driveWithVoltage(Volts.of(ElevatorConstants.slowVoltageDown)));
    }

    public Command stop() {
        return run(() -> {
            driveWithVoltage(Volts.of(0));
            followerMotor.setControl(new Follower(9, true));
        });
    }

    public Command zeroElevatorDown() {
        return run(this::goToFloorHeightCommand).until(() -> isRotationsAlmostAtZero()).andThen(driveWithSlowVoltageDown())
        .until(() -> { return isAtExtrema() || !lowerDigitalInput.get(); }).andThen(stop());
    }

    public Command autoHoneDown() {
        return driveWithSlowVoltageDown()
            .until(this::isAtExtrema).andThen(resetPose()).andThen(stop().withTimeout(0.1));
    }

    public Command autoHonePose() {
        return driveWithSlowVoltageUp().withTimeout(0.5).andThen(autoHoneDown()).withName("Auto Hone");
    }

    public Command resetPose() {
        return runOnce(() -> {
            elevatorMotorLeft.setControl(new VoltageOut(0).withLimitReverseMotion(true));
            followerMotor.setControl(new VoltageOut(0).withLimitReverseMotion(true));
            setFollowerMode();
        });
    }

    public boolean isRotationsAlmostAtMax() {
        return elevatorMotorLeft.getPosition().getValueAsDouble() >= ElevatorConstants.rotationsBeforeMaxHeight;
    }

    public Command driveWithSlowVoltageUp() {
        return run(() -> driveWithVoltage(Volts.of(ElevatorConstants.slowVoltageUp)));
    }

    // this command will definently be changed due to how the elevator needs to be slowed down
    public Command zeroElevatorUp() {
        return run(this::goToBargeHeightCommand).until(() -> isRotationsAlmostAtMax()).andThen(driveWithSlowVoltageUp())
        .until(() -> { return isAtExtrema() || !upperDigitalInput.get(); }).andThen(stop());
    }

    UltraDoubleLog setPose = new UltraDoubleLog("Elevator/Set Rotations");
    UltraDoubleLog currentPose = new UltraDoubleLog("Elevator/Current Rotations");
    StatusSignal<Double> setPoseSignal = elevatorMotorLeft.getClosedLoopReference();
    StatusSignal<Angle> currentPoseSignal = elevatorMotorLeft.getPosition();


    public Command runSpeed(DoubleSupplier speed) {
        if (estop) {
            return run(() -> { elevatorMotorLeft.set(0);});
        }

        return run(() -> elevatorMotorLeft.set(speed.getAsDouble() * 0.1));
    }

    @Override
    public void periodic() {
        estopAlert.set(estop);

        StatusSignal.refreshAll(currentPoseSignal);
        double currentPoseTHing = currentPoseSignal.getValueAsDouble();
        if (!upperDigitalInput.get() && poseToHold > currentPoseTHing) {
            // estop = true;
            // setDefaultCommand(stop());
            // elevatorMotorLeft.set(0);
            extendArm(currentPoseTHing);
        }

        leftVelocityPub.update();
        leftTempPub.update();
        leftPosePub.update();
        rightVelocityPub.update();
        rightTempPub.update();
        rightPosePub.update();

        String name = "None";
        Command cmd = this.getCurrentCommand();
        if (cmd != null) {
            name = cmd.getName();
        }
        commandRunningPublisher.update(name);

        SmartDashboard.putBoolean("Upper Limit", !upperDigitalInput.get());
        SmartDashboard.putBoolean("Lower Limit", !lowerDigitalInput.get());

        BaseStatusSignal.refreshAll(setPoseSignal, currentPoseSignal);
        setPose.update(setPoseSignal.getValue());
        currentPose.update(currentPoseSignal.getValueAsDouble());
        if (RobotBase.isReal()) {
            // elevator.setLength(getElevatorHeight().in(Meters));
        }
        
        noelevAlert.set(!elevatorMotorLeft.isAlive());
        
        // if (!lowerDigitalInput.get() || !upperDigitalInput.get()) {
        //     driveWithVoltage(Volts.of(0));
        // }
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
