package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.BotType;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX elevatorMotorLeft;
    private final TalonFX elevatorMotorRight;

    private final DigitalInput magneticLimitSwitch;

    private final PositionTorqueCurrentFOC elevatorPositionControl = new PositionTorqueCurrentFOC(Degrees.of(0));
    private final VoltageOut elevatorVoltageControl = new VoltageOut(0).withEnableFOC(true);

    private final StatusSignal<AngularVelocity> leftMotorVelocity;
    private final StatusSignal<Angle> leftMotorPosition;
    private final StatusSignal<Double> leftMotorSetPositon;
    private final StatusSignal<Temperature> leftMotorTemperature;
    private final StatusSignal<Current> leftMotorStatorCurrent;

    private final StatusSignal<AngularVelocity> rightMotorVelocity;
    private final StatusSignal<Angle> rightMotorPosition;
    private final StatusSignal<Temperature> rightMotorTemperature;
    private final StatusSignal<Current> rightMotorStatorCurrent;

    public ElevatorIOTalonFX(BotType bot, int leftMotorID, int rightMotorID, int limitSwitchID) {
        elevatorMotorLeft = new TalonFX(leftMotorID);
        elevatorMotorRight = new TalonFX(rightMotorID);
        magneticLimitSwitch = new DigitalInput(limitSwitchID);

        elevatorMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotorRight.setNeutralMode(NeutralModeValue.Brake);

        elevatorPositionControl.withSlot(bot.slotId);

        elevatorMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotorRight.setNeutralMode(NeutralModeValue.Brake);

        elevatorMotorLeft.getConfigurator().apply(ElevatorConstants.realBotConfigs);
        elevatorMotorLeft.getConfigurator().apply(ElevatorConstants.alphaBotConfigs);
        elevatorMotorRight.getConfigurator().apply(ElevatorConstants.realBotConfigs);
        elevatorMotorRight.getConfigurator().apply(ElevatorConstants.alphaBotConfigs);

        elevatorMotorRight.setControl(new Follower(leftMotorID, true));

        leftMotorVelocity = elevatorMotorLeft.getVelocity();
        leftMotorPosition = elevatorMotorLeft.getPosition();
        leftMotorSetPositon = elevatorMotorLeft.getClosedLoopReference();
        leftMotorTemperature = elevatorMotorLeft.getDeviceTemp();
        leftMotorStatorCurrent = elevatorMotorLeft.getStatorCurrent();

        rightMotorVelocity = elevatorMotorRight.getVelocity();
        rightMotorPosition = elevatorMotorRight.getPosition();
        rightMotorTemperature = elevatorMotorRight.getDeviceTemp();
        rightMotorStatorCurrent = elevatorMotorRight.getStatorCurrent();

        StatusSignal.setUpdateFrequencyForAll(
            50,
            leftMotorVelocity,
            leftMotorPosition,
            leftMotorSetPositon,
            leftMotorTemperature,
            leftMotorStatorCurrent,
            rightMotorVelocity,
            rightMotorPosition,
            rightMotorTemperature,
            rightMotorStatorCurrent
        );
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.leftMotorAlive = elevatorMotorLeft.isAlive();
        inputs.leftVelocity = leftMotorVelocity.getValueAsDouble();
        inputs.leftPosition = leftMotorPosition.getValueAsDouble();
        inputs.leftSetPoint = leftMotorSetPositon.getValueAsDouble();
        inputs.leftTemperature = leftMotorTemperature.getValueAsDouble();
        inputs.leftStatorCurrent = leftMotorStatorCurrent.getValueAsDouble();

        inputs.rightMotorAlive = elevatorMotorRight.isAlive();
        inputs.rightVelocity = rightMotorVelocity.getValueAsDouble();
        inputs.rightPosition = rightMotorPosition.getValueAsDouble();
        inputs.rightTemperature = rightMotorTemperature.getValueAsDouble();
        inputs.rightStatorCurrent = rightMotorStatorCurrent.getValueAsDouble();

        inputs.limitSwitchState = magneticLimitSwitch.get();
    }

    @Override
    public void runVoltage(Voltage volts) {
        elevatorMotorLeft.setControl(elevatorVoltageControl.withOutput(volts));
    }
}
