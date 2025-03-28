package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ultralogger.UltraDoubleLog;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TrigVisionSubsystem;

public class AlignBarge extends Command {
    private final TrigVisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier horizontalInputSupplier;

    private final ProfiledPIDController profiledPIDController = new ProfiledPIDController(AutoConstants.Turbo.kTranslationXP, AutoConstants.Turbo.kTranslationXI, AutoConstants.Turbo.kTranslationXD, new TrapezoidProfile.Constraints(100, 10));

    private final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle().withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    
    public AlignBarge(TrigVisionSubsystem vision, CommandSwerveDrivetrain drivetrain, DoubleSupplier horizontalInputSupplier) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.horizontalInputSupplier = horizontalInputSupplier;

        addRequirements(drivetrain);

        driveRequest.HeadingController.setPID(AutoConstants.Turbo.kRotationP, AutoConstants.Turbo.kRotationI, AutoConstants.Turbo.kRotationD);
        driveRequest.HeadingController.setTolerance(0.026);
    }

    private double sign = 1;

    @Override
    public void execute() {
        Optional<Translation2d> translationOptional = vision.getBargePose();
        double pidOutput = 0;
        if (!translationOptional.isEmpty()) {
            double distance = translationOptional.get().getMeasureX().in(Meters);
            SmartDashboard.putNumber("distance x", distance);
            pidOutput = -profiledPIDController.calculate(distance, AutoConstants.distanceFromBarge.in(Meters));
        }

        Optional<Long> tagID = vision.getBargeTagID();
        if (tagID.isPresent()) {
            if (tagID.get() == 4 || tagID.get() == 5) {
                sign = -1;
            } else {
                sign = 1;
            }
        }

        if (profiledPIDController.atGoal()) {
            vision.isAlignedTimestamp = RobotController.getFPGATime();
        }

        driveRequest.withVelocityX(Meters.of(sign * pidOutput).per(Second));
        driveRequest.withVelocityY(horizontalInputSupplier.getAsDouble());
        driveRequest.withTargetDirection(Rotation2d.fromDegrees(sign * -90));

        drivetrain.setControl(driveRequest);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    
}
