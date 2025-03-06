package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TrigVisionSubsystem;

public class AlignBarge extends Command {
    private final TrigVisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier horizontalInputSupplier;

    private final PIDController translationPIDController = new PIDController(AutoConstants.Turbo.kTranslationP, AutoConstants.Turbo.kTranslationI, AutoConstants.Turbo.kTranslationD);
    private final PIDController rotationPIDController = new PIDController(AutoConstants.Turbo.kRotationP, AutoConstants.Turbo.kRotationI, AutoConstants.Turbo.kRotationD);

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    public AlignBarge(TrigVisionSubsystem vision, CommandSwerveDrivetrain drivetrain, DoubleSupplier horizontalInputSupplier) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.horizontalInputSupplier = horizontalInputSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Optional<Distance> distanceOptional = vision.distanceToBarge();
        if (distanceOptional.isEmpty()) return;
        Distance distance = distanceOptional.get();

        // driveRequest.withRotationalRate(Rotations.of(rotationPIDController.calculate(orientation.in(Rotations), 0)).per(Second));
        driveRequest.withVelocityX(Meters.of(-translationPIDController.calculate(distance.in(Meters), AutoConstants.targetDistanceFromBarge.in(Meters))).per(Second));
        driveRequest.withVelocityY(horizontalInputSupplier.getAsDouble());
        driveRequest.withRotationalRate(Rotations.of(rotationPIDController.calculate(drivetrain.getPigeon2().getYaw().getValue().in(Rotations), 0.25)).per(Second));

        drivetrain.setControl(driveRequest);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    
}
