package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED led = new AddressableLED(LEDConstants.ledID);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.ledLength);

    public LEDSubsystem() {
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    public Command solidColor(Color color) {
        return applyPattern(LEDPattern.solid(color));
    }

    public Command fauxRSL() {
        // Pink
        LEDPattern pattern = LEDPattern.solid(new Color(255, 20, 20));

        if (RobotBase.isReal()) {
            pattern = pattern.synchronizedBlink(RobotController::getRSLState);
        } else {
            pattern = pattern.blink(Time.ofBaseUnits(1, Units.Second));
        }
         
        return applyPattern(pattern);
    }

    private LEDPattern allianceColorGetter() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()) {
            // Pink
            return LEDPattern.solid(new Color(255, 20, 20));
        } else if (alliance.get().equals(Alliance.Red)) {
            // Red
            return LEDPattern.solid(new Color(255, 0, 0));
        } else {
            // Blue
            return LEDPattern.solid(new Color(0, 0, 255));
        }
    }

    public Command allianceColor() {
        return applyPattern(this::allianceColorGetter);
    }

    public Command applyPattern(Supplier<LEDPattern> patternSupplier) {
        return this.run(() -> {
            patternSupplier.get().applyTo(ledBuffer);
            led.setData(ledBuffer);
        }).ignoringDisable(true);
    }

    public Command applyPattern(LEDPattern pattern) {
        return this.run(() -> {
            pattern.applyTo(ledBuffer);
            led.setData(ledBuffer);
        }).ignoringDisable(true);
    }
}
