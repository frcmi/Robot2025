package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TrigVisionSubsystem;

public class AlignReef extends Command {
    private final TrigVisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;

    private final ProfiledPIDController profiledPIDControllerX = new ProfiledPIDController(AutoConstants.Turbo.kTranslationXP, AutoConstants.Turbo.kTranslationXI, AutoConstants.Turbo.kTranslationXD, new TrapezoidProfile.Constraints(10, 1));
    private final PIDController PIDControllerY = new PIDController(AutoConstants.Turbo.kTranslationYP, AutoConstants.Turbo.kTranslationYI, AutoConstants.Turbo.kTranslationYD);
    private final DoubleSupplier xSupplier;
    private final Rev2mDistanceSensor distance;
    private final boolean autoX;

    private final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle().withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public AlignReef(TrigVisionSubsystem vision, CommandSwerveDrivetrain swerve, Rev2mDistanceSensor distanceSensor, DoubleSupplier xDoubleSupplier, boolean autoX) {
        this.autoX = autoX;
        distance = distanceSensor;
        xSupplier = xDoubleSupplier;

        this.vision = vision;
        drivetrain = swerve;

        addRequirements(drivetrain);

        driveRequest.HeadingController.setPID(AutoConstants.Turbo.kRotationP, AutoConstants.Turbo.kRotationI, AutoConstants.Turbo.kRotationD);
    }

    @Override
    public void initialize() {
        profiledPIDControllerX.setGoal(AutoConstants.distanceFromReef.in(Meters));
        PIDControllerY.setSetpoint(AutoConstants.sidewaysDistanceFromReef.in(Meters));
        PIDControllerY.setTolerance(0.025);
    }

    @Override
    public void execute() {
        Optional<Translation2d> translationOptional = vision.getReefPose();
        
        double pidOutputX = xSupplier.getAsDouble();
        double pidOutputY = 0;

        if (translationOptional.isPresent()) {
            // double distanceX = translationOptional.get().getMeasureX().in(Meters);
            // pidOutputX = -profiledPIDController.calculate(distanceX, AutoConstants.distanceFromReef.in(Meters));

            double distanceY = translationOptional.get().getMeasureY().in(Meters);
            pidOutputY = -PIDControllerY.calculate(distanceY);
        }

        SmartDashboard.putBoolean("Y at setpoint", PIDControllerY.atSetpoint());

        if (PIDControllerY.atSetpoint()) { // && profiledPIDControllerX.atGoal()
            vision.isAlignedTimestamp = RobotController.getFPGATime();
            double distanceMeasurement = distance.getRange();
            if (autoX) {
                if (distanceMeasurement != -1) {
                    pidOutputX = profiledPIDControllerX.calculate(Inches.of(distanceMeasurement).in(Meters));
                } else {
                    pidOutputX = profiledPIDControllerX.calculate(2);
                }
            }
        }

        driveRequest.withVelocityX(Meters.of(pidOutputX).per(Second));
        driveRequest.withVelocityY(Meters.of(pidOutputY).per(Second));
        driveRequest.withTargetDirection(Rotation2d.fromDegrees(90));

        drivetrain.setControl(driveRequest);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    
}
