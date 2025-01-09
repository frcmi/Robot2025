package frc.robot.subsystems;

import java.beans.Encoder;
import java.util.concurrent.Flow.Publisher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    // this is the motor that will extend the elevator
    private final TalonFX extendingMotor = new TalonFX(0);
    
    // will track the position of the elevator
    private final Encoder encoder;
    
    // will figure out dimensions later
    private final Mechanism2d elevator;
    
    // limit switch for when the elevator is not extended
    private final DigitalInput limitSwitch;

    // there will be at least one limit switch and an encoder to track the position of the elevator
    public ElevatorSubsystem(DigitalInput limitSwitch, Mechanism2d elevator, Encoder encoder){
        this.encoder = encoder;
        this.elevator = elevator;
        this.limitSwitch = limitSwitch;
        extendingMotor.setNeutralMode(NeutralModeValue.Brake);
    }
   
    // I would assume that there is only going to be one motor to extend the elevator but we will see
    public Command extendArm(double speed){
        return run(() -> {
            extendingMotor.set(speed);
        });
    }
    // the speed of the motor will depend on how it is set up and how much power is needed
    public Command lowerArm(double speed){
        return run(() -> {
         if(limitSwitch.get()){
            extendingMotor.set(0.0);
         } else{
            extendingMotor.set(-speed);
         }  
        });
    }
}
