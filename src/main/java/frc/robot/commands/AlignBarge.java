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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TrigVisionSubsystem;

public class AlignBarge extends Command {
    private final TrigVisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier horizontalInputSupplier;

    private final PIDController translationPIDController = new PIDController(AutoConstants.Turbo.kTranslationP, AutoConstants.Turbo.kTranslationI, AutoConstants.Turbo.kTranslationD);

    private final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle().withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public AlignBarge(TrigVisionSubsystem vision, CommandSwerveDrivetrain drivetrain, DoubleSupplier horizontalInputSupplier) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.horizontalInputSupplier = horizontalInputSupplier;

        addRequirements(drivetrain);

        driveRequest.HeadingController.setPID(AutoConstants.Turbo.kRotationP, AutoConstants.Turbo.kRotationI, AutoConstants.Turbo.kRotationD);
    }

    @Override
    public void execute() {
        Optional<Distance> distanceOptional = vision.getLateralDistanceToBarge();
        if (distanceOptional.isEmpty()) return;
        Distance distance = distanceOptional.get();

        double pidOutput = -translationPIDController.calculate(distance.in(Meters), AutoConstants.targetDistanceFromBarge.in(Meters));

        SmartDashboard.putNumber("Auto Align Error", translationPIDController.getError());
        SmartDashboard.putBoolean("Auto Align Close Enough", translationPIDController.atSetpoint());

        if (translationPIDController.atSetpoint()) {
            vision.isAlignedTimestamp = RobotController.getFPGATime();
        }
        

        driveRequest.withVelocityX(Meters.of(pidOutput).per(Second));
        driveRequest.withVelocityY(horizontalInputSupplier.getAsDouble());
        driveRequest.withTargetDirection(Rotation2d.fromDegrees(-90));

        drivetrain.setControl(driveRequest);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    
}
