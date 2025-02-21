package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;

public final class Constants {
  public enum BotType {
    MAIN_BOT(0),
    ALPHA_BOT(1),
    SIM_BOT(2),
    REPLAY_BOT(-2);

    public final int slotId;

    private BotType(int slotId) {
      this.slotId = slotId;
    }
  }

  public static boolean replay = false;

  public static class RobotDetectionConstants {
    // Mac addresses of rios, google it or ask brandon
    public static final String mainBotMacAddress = "00:80:2F:40:6D:81";
    public static final String alphaBotMacAddress = "00:80:2F:39:0D:E5";
  }

  public static class TelemetryConstants {
    // DON'T ENABLE UNLESS ABSOLUTELY NEEDED
    // this will fully disable logging even when FMS is connected.
    public static final boolean killswitch = false;
    // If true, data won't be sent over network even when not connected to FMS
    public static final boolean disableNetworkLogging = false;
    // ONLY ENABLE IN DEV (this *should* be overwritten when connected to FMS, but that's untested)
    public static final boolean disableDatalog = false;
    // Prefix in NetworkTables, must end with a '/'
    public static final String tabPrefix = "UltraLog/";
    // How often to re-check if the FMS is connected (and disable network logging if so)
    public static final double fmsCheckDelay = 1000;
  }

  public static class ClawConstants {
    public static final int beambreakID = 1;
    public static final int motorControllerID = 23;
    public static final double intakeSpeed = -0.5;
    public static final double shootSpeed = 1;
    public static final double stopSpeed = 0.0;
  }

  public static class ClimberCostansts {
    public static final int climberMotorID = 0;
  }

  public static class LEDConstants {
    public static final int ledID = 0;
    public static final int ledLength = 10;
  }

  public static class PivotConstants {
    public static final int motorID = 13;
    public static final int encoderID = 0; // TODO: make this
    // The "zero" angle is forward, larger values will rotate upwards
    public static final Angle floorAngle = Degrees.of(0);
    public static final Angle onCoralAngle = Degrees.of(20);
    public static final Angle reefOneAngle = Degrees.of(45);
    public static final Angle reefTwoAngle = Degrees.of(45);
    public static final Angle bargeAngle = Degrees.of(90);

    public static final Angle maxAngle = Degrees.of(5000);
    public static final Angle minAngle = Degrees.of(-1000.5);
    public static final double maxVelocity = 0.0;
    public static final double maxAccel = 0.0;

    public static final double gearRatio = 0.6;
    public static final double momentOfInertia = 0.001;
    public static final double length = 0.5;

    public static final double kRealBotP = 1.15;
    public static final double kRealBotI = 0.0;
    public static final double kRealBotD = 0.0;

    public static final double kRealBotS = 0.13;
    public static final double kRealBotV = 0.0;
    public static final double kRealBotA = 0.0;
    public static final double kRealBotG = 0.52;

    public static final double kSimulationBotP = 0.02;
    public static final double kSimulationBotI = 0.0;
    public static final double kSimulationBotD = 0.0;

    public static final double kSimulationBotS = 0.0;
    public static final double kSimulationBotV = 0.0;
    public static final double kSimulationBotA = 0.0;
    public static final double kSimulationBotG = 0.08293;
  }

  public static class ElevatorConstants {
    public static final int upperLimitSwitchID = 3;
    public static final int lowerLimitSwitchID = 4;

    public static final int leftMotorID = 9;
    public static final int rightMotorID = 10;

    // TODO: double check
    public static final Angle absoluteBottom = Rotations.of(-0.4);
    public static final Angle absoluteTop = Rotations.of(57.3);
    public static final boolean absoluteCinema = true;

    public static final double rotationsBeforeZero = 1.5;
    // this is a place holder until we can figure out what the real number of rotation is
    public static final double rotationsBeforeMaxHeight = 0.0;
    public static final double floorHeight = rotationsBeforeZero - 0.5;
    public static final double onCoralHeight = 25;
    public static final double reefOneHeight = 30;
    public static final double reefTwoHeight = 40;
    public static final double bargeHeight = 57;
    public static final double slowVoltageDown = -0.32;
    public static final double slowVoltageUp = 1.5;

    public static final int slotId = 1;

    public static final Slot0Configs realBotConfigs =
        new Slot0Configs()
            .withKP(0.0)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKG(0.0)
            .withGravityType(GravityTypeValue.Elevator_Static);

    public static final Slot1Configs alphaBotConfigs =
        new Slot1Configs()
            .withKP(0.44)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.068905)
            .withKV(0.12637)
            .withKA(0.0018766)
            .withKG(0.51897)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    public static final Slot1Configs simBotConfigs =
        new Slot1Configs()
            .withKP(6)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0)
            .withKV(0)
            .withKA(0)
            .withKG(0.1265)
            .withGravityType(GravityTypeValue.Elevator_Static);

    // TODO: get conversion value
    public static final double rotationsPerMeter = 29; // ROTATIONS OF OUTPUT GEAR, NOT MOTOR
    public static final double gearRatio = 8.0 / 1.0;
    public static final double elevatorInertia = 0.001; // In Kg Meters^2
    public static final double minElevatorHeight = 0.2; // FOR SIM!!!
    public static final double maxElevatorHeight = 2.2;
    public static final double elevatorWeight = 1.814;
    public static final double drumRadius = 0.04445;
  }
}
