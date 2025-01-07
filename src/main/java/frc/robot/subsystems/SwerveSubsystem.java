package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Robot;

public final class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {
    public static SwerveSubsystem configure() {
        var factory = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(6.746031746031747)
                .withSteerMotorGearRatio(21.428571428571427)
                .withWheelRadius(2)
                .withSlipCurrent(300)
                .withSteerMotorGains(new Slot0Configs()
                        .withKP(100)
                        .withKI(0)
                        .withKD(0.2)
                        .withKS(0)
                        .withKV(1.5)
                        .withKA(0))
                .withDriveMotorGains(new Slot0Configs()
                        .withKP(3)
                        .withKI(0)
                        .withKD(0)
                        .withKS(0)
                        .withKV(0)
                        .withKA(0))
                .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withSpeedAt12Volts(4.73)
                .withSteerInertia(1.0e-5)
                .withDriveInertia(0.001)
                .withSteerFrictionVoltage(0.25)
                .withDriveFrictionVoltage(0.25)
                .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                .withCouplingGearRatio(3.5714285714285716);

        var swerveConfig = new SwerveDrivetrainConstants()
                .withPigeon2Id(0)
                .withCANBusName("");

        double moduleX = Units.inchesToMeters(11.375);
        double moduleY = Units.inchesToMeters(11.44);

        var moduleConfigs = new SwerveModuleConstants[] {
                // front left
                factory.createModuleConstants(5, 1, 9, 0.49658203125, moduleX, moduleY, false, false, false),

                // front right
                factory.createModuleConstants(6, 2, 10, 0.311767578125, moduleX, -moduleY, false, true, true),

                // back left
                factory.createModuleConstants(7, 3, 11, -0.211669921875, -moduleX, moduleY, false, false, false),

                // back right
                factory.createModuleConstants(8, 4, 12, 0.0732421875, -moduleX, -moduleY, false, true, true)
        };

        return new SwerveSubsystem(swerveConfig, moduleConfigs);
    }

    public SwerveSubsystem(SwerveDrivetrainConstants constants, SwerveModuleConstants... modules) {
        super(constants, modules);
        if (Robot.isSimulation()) {
            startSimThread();
        }
    }

    public SwerveSubsystem(SwerveDrivetrainConstants constants, double odometryFrequency,
            SwerveModuleConstants... modules) {
        super(constants, odometryFrequency, modules);
        if (Robot.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> setControl(request.get()));
    }

    private void startSimThread() {
        m_LastSimUpdate = Utils.getCurrentTimeSeconds();
        m_SimThread = new Notifier(() -> {
            double time = Utils.getCurrentTimeSeconds();
            double delta = time - m_LastSimUpdate;
            m_LastSimUpdate = time;

            updateSimState(delta, RobotController.getBatteryVoltage());
        });

        m_SimThread.startPeriodic(0.005); // 5 ms
    }

    private double m_LastSimUpdate;
    private Notifier m_SimThread;
}
