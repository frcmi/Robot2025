package frc.robot;

import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

public final class Constants {
    public enum BotType {
        MAIN_BOT(0),
        ALPHA_BOT(1),
        SIM_BOT(2);

        public final int slotId;

        private BotType(int slotId) {
            this.slotId = slotId;
        }
    }

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
        public static final int beambreakChannel = 1;
        public static final int motorControllerID = 23;
        public static final double intakeSpeed = -0.5;
        public static final double shootSpeed = 0.5;
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
        // TODO: figure out real values
        // The "zero" angle is forward, larger values will rotate upwards
        public static final Angle floorAngle = Degrees.of(0);
        public static final Angle onCoralAngle = Degrees.of(20);
        public static final Angle reefOneAngle = Degrees.of(45);
        public static final Angle reefTwoAngle = Degrees.of(45);
        public static final Angle bargeAngle = Degrees.of(90);

        public static final Angle maxAngle = Degrees.of(5);
        public static final Angle minAngle = Degrees.of(0);
        public static final double maxVelocity = 0.0;
        public static final double maxAccel = 0.0;

        public static final double kP = 1.15;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kS = 0.13;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kG = 0.52;
    }

    public static class ElevatorConstants {
        public static final int magneticLimitSwitchID = 4;

        public static final double elevatorMidpoint = 25d; // does not need to be precise

        public static final int absoluteEncoderChannel = 0;
        public static final int limitSwitchChannel = 1;

        public static final Angle absoluteEncoderOffset = Degrees.of(0);

        public static final double rotationsBeforeZero = 1.5;
        // this is a place holder until we can figure out what the real number of rotation is
        public static final double rotationsBeforeMaxHeight = 0.0;
        public static final double floorHeight = rotationsBeforeZero - 0.5;
        public static final double onCoralHeight = 0.3;
        public static final double reefOneHeight = 0.8;
        public static final double reefTwoHeight = 1.0;
        public static final double bargeHeight = 1.6;
        public static final double slowVoltageDown = -0.32;
        public static final double slowVoltageUp = 1.5;

        public static final int slotId = 1;

        public static final Slot0Configs realBotConfigs = new Slot0Configs()
            .withKP(0.0)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKG(0.0)
            .withGravityType(GravityTypeValue.Elevator_Static);

        public static final Slot1Configs alphaBotConfigs = new Slot1Configs()
            .withKP(0.2)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.087818)
            .withKV(0.11882)
            .withKA(0.0018582)
            .withKG(0.31307)
            .withGravityType(GravityTypeValue.Elevator_Static);

        // TODO: get conversion value(if this isn't it)
        public static final double rotationsPerMeter = 2; // ROTATIONS OF OUTPUT GEAR, NOT MOTOR 
        public static final double gearRatio = 8.0/1.0;
        public static final double elevatorInertia = 0.001; // In Kg Meters^2 
        public static final double minElevatorHeight = 0.2; // FOR SIM!!!
        public static final double maxElevatorHeight = 2.2;
        public static final double elevatorWeight = 1.814;
        public static final double drumRadius = 0.04445;
    }
}
