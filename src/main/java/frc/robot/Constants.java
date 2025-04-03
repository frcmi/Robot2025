package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

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

    public static class AutoConstants {
        public static int[] bargeTagIDs = { 4, 5, 14, 15 };
        public static int[] reefTagIDs = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };
        public static final Angle cameraAngle = Degrees.of(60);
        public static final Distance distanceFromBarge = Inches.of(20.0);
        public static final Distance distanceFromReef = Meters.of(0.3);
        public static final Distance sidewaysDistanceFromReef = Meters.of(0.12);
        public static final Time lastPoseTimeout = Milliseconds.of(50);
        public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
        public static double MaxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        public static double MaxAcceleration = 25.0;
        public static class Turbo {
            public static final double kRotationP = 5.0;
            public static final double kRotationI = 0.0;
            public static final double kRotationD = 0.0;
    
            public static final double kTranslationXP = 2.1;
            public static final double kTranslationXI = 0.0;
            public static final double kTranslationXD = 0.0;
    
            public static final double kTranslationYP = 4.5;
            public static final double kTranslationYI = 0.1;
            public static final double kTranslationYD = 0.0;
        }
        public static class Alpha {
            public static final double kdP = 1.0;
            public static final double kdI = 0.0;
            public static final double kdD = 0.0;
    
            public static final double ktP = 1.0;
            public static final double ktI = 0.0;
            public static final double ktD = 0.0;
        }
    }

    public static class ClawConstants {
        public static final int beambreakChannel = 1;
        public static final int motorControllerID = 23;
        public static final double intakeSpeed = -0.75;
        public static final double shootSpeed = 0.8;
        // TODO: Figure out good speed for shooting at the processor
        public static final double processorShootSpeed = 0.3;
        public static final double stopSpeed = -0.04;
    }

    public static class ClimberCostansts {
        public static final int climberMotorID = 15;
    }

    public static class LEDConstants {
        public static final int ledID = 0;
        public static final int ledLength = 20;
    }

    public static class PivotConstants {
        public static final int motorID = 13;
        // The "zero" angle is right, larger values will rotate upwards
        public static final Angle floorAngle = Rotations.of(0.05876435021910875);
        public static final Angle onCoralAngle = Rotations.of(0.13607985215199636);
        public static final Angle reefOneAngle = Rotations.of(0.13396100209902506);
        public static final Angle reefTwoAngle = Rotations.of(0.14660425241510633);
        public static final Angle processorAngle = Rotations.of(0.13322460283061502);
        public static final Angle bargeAngle = Rotations.of(0.3);
        public static final Angle stowAngle = Rotations.of(0.27261940556548514);
        public static final Angle coralIntake = Rotations.of(-0.059247502731187396);

        public static final Angle maxAngle = Degrees.of(5);
        public static final Angle minAngle = Degrees.of(0);
        public static final double maxVelocity = (15) * (44 / 12) * 15d;
        public static final double maxAccel = maxVelocity * 2.5;

        public static class AlphaBot {  
            public static final double kP = 1.15;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            public static final double kS = 0.17;
            public static final double kG = 0.548;

            public static final double offset = 0.05 - 0.75 - 0.3885356534633913 - 0.5 + 1;
            public static final double discontinuity = -0.3;
        }

        public static class TurboBot {
            public static final double kP = 1.5d;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            public static final double kS = 0.26;
            public static final double kG = 0.505; // 0.4355

            public static final double offset = -0.233154296875;
            public static final double discontinuity = 0.75;
        }
    }

    public static class ElevatorConstants {
        public static final int upperLimitSwitchID = 3;
        public static final int lowerLimitSwitchID = 4;

        public static final Angle absoluteBottom = Rotations.of(-0.4);
        public static final Angle absoluteTop = Rotations.of(60);
        public static final boolean absoluteCinema = absoluteTop.gt(absoluteBottom);

        public static final double rotationsBeforeZero = 1.5;
        // this is a place holder until we can figure out what the real number of rotation is
        public static final double rotationsBeforeMaxHeight = 0.0;
        public static final double floorHeight = 0.99169921875;
        public static final double stowHeight = 0.2;
        public static final double onCoralHeight = floorHeight;
        public static final double reefOneHeight = 15.19 + 2;
        public static final double reefTwoHeight = 29.85 + 2;
        public static final double bargeHeight = 61.1;
        public static final double coralIntake = 40.23876953125;
        public static final double slowVoltageDown = -0.32;
        public static final double slowVoltageUp = 1.5;

        public static final int slotId = 1;

        public static final Slot0Configs realBotConfigs = new Slot0Configs()
            .withKP(0.7)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.16078)
            .withKV(0.12648)
            .withKA(0.0027715)
            .withKG(0.58887)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

        public static final Slot1Configs alphaBotConfigs = new Slot1Configs()
            .withKP(0.44)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.068905)
            .withKV(0.12637)
            .withKA(0.0018766)
            .withKG(0.51897)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

        // TODO: figure out real conversion and heights
        public static final double inchesPerRotation = 20;

        public static final double rotationsPerMeter = 1; // ROTATIONS OF OUTPUT GEAR, NOT MOTOR 
        public static final double gearRatio = 8.0/1.0;
        public static final double elevatorInertia = 0.001; // In Kg Meters^2 
        public static final double minElevatorHeight = 0.2; // FOR SIM!!!
        public static final double maxElevatorHeight = 2.2;
        public static final double elevatorWeight = 1.814;
        public static final double drumRadius = 0.04445;
    }
}
