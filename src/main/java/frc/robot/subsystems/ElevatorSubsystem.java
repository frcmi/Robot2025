package frc.robot.subsystems;

import java.beans.Encoder;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {
    // this is the motor that will extend the elevator
    private final TalonFX extendingMotor = new TalonFX(0);

    private final Slot0Configs configs = new Slot0Configs()
    .withKP(1.0)
    .withKI(0.0)
    .withKD(0.0)
    .withKS(0.0)
    .withKV(0.0)
    .withKA(0.0)
    .withKG(0.0);
    // will track the position of the elevator
    private final Encoder encoder;
    
    // will figure out dimensions later
    private final Mechanism2d elevator;
    
    // limit switch for when the elevator is not extended
    private final DigitalInput limitSwitch;

    PositionTorqueCurrentFOC elevatorPositionControl = new PositionTorqueCurrentFOC(Degrees.of(0)).withSlot(0);

    // there will be at least one limit switch and an encoder to track the position of the elevator
    public ElevatorSubsystem(DigitalInput limitSwitch, Mechanism2d elevator, Encoder encoder){
        this.encoder = encoder;
        this.elevator = elevator;
        this.limitSwitch = limitSwitch;
        extendingMotor.setNeutralMode(NeutralModeValue.Brake);
        extendingMotor.getConfigurator().apply(configs);
    }

    // I would assume that there is only going to be one motor to extend the elevator but we will see
    public Command extendArm(double rotations){
        return runOnce(() -> {
            extendingMotor.setControl(elevatorPositionControl.withPosition(rotations));
            
        });
    }

    public Command goToFloorHeightCommand() {
        return (extendArm(ElevatorConstants.floorHeight/ElevatorConstants.inchesPerRotation));
    }

    public Command goOnCoralHeightCommand() {
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
