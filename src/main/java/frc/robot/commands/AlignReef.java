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
import edu.wpi.first.math.filter.Debouncer;
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

    private final ProfiledPIDController profiledPIDControllerX = new ProfiledPIDController(AutoConstants.Turbo.kTranslationXP, AutoConstants.Turbo.kTranslationXI, AutoConstants.Turbo.kTranslationXD, new TrapezoidProfile.Constraints(0.5, 0.1));
    private final PIDController PIDControllerY = new PIDController(AutoConstants.Turbo.kTranslationYP, AutoConstants.Turbo.kTranslationYI, AutoConstants.Turbo.kTranslationYD);
    private final DoubleSupplier xSupplier;
    private final Rev2mDistanceSensor distance;
    private final boolean autoX;
    private final Debouncer isAlignedDebouncer = new Debouncer(0.25);
    private final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle().withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    private Optional<int[]> limitTag = Optional.empty();
    private boolean forcedLimit = false;
    public AlignReef(TrigVisionSubsystem vision, CommandSwerveDrivetrain swerve, Rev2mDistanceSensor distanceSensor, DoubleSupplier xDoubleSupplier, boolean autoX, Optional<int[]> limitTag) {
        this.autoX = autoX;
        this.limitTag = limitTag;
        if (limitTag.isPresent()) {
            forcedLimit = true;
        }
        distance = distanceSensor;
        xSupplier = xDoubleSupplier;

        this.vision = vision;
        drivetrain = swerve;

        addRequirements(drivetrain);

        driveRequest.HeadingController.setPID(AutoConstants.Turbo.kRotationP, AutoConstants.Turbo.kRotationI, AutoConstants.Turbo.kRotationD);
    }

    public double timestampStart = 0;

    @Override
    public void initialize() {
        profiledPIDControllerX.setGoal(AutoConstants.distanceFromReef.in(Meters));
        PIDControllerY.setSetpoint(AutoConstants.sidewaysDistanceFromReef.in(Meters));
        SmartDashboard.putNumber("Reef Setpoint", AutoConstants.sidewaysDistanceFromReef.in(Meters));
        PIDControllerY.setTolerance(0.025);

        timestampStart = RobotController.getFPGATime();

        isAlignedForAuto = false;
        isAlignedDebouncer.calculate(false);

        if (!forcedLimit) {
            limitTag = Optional.empty();
        }
    }

    int tagID = 18;

    boolean isAlignedForAuto = false;

    @Override
    public void execute() {
        Optional<Translation2d> translationOptional = vision.getReefPose();
        
        if (vision.lastSeenReefTag.isPresent()) {
            int tmpTag = vision.lastSeenReefTag.get().tid;
            if (vision.tagIsInArray(tmpTag, limitTag.orElseGet(() -> new int[] { tmpTag }))) {
                tagID = tmpTag;
            } else {
                translationOptional = Optional.empty();
            }
        }
        
        double pidOutputX = xSupplier.getAsDouble();
        double pidOutputY = 0;

        if (translationOptional.isPresent()) {
            // double distanceX = translationOptional.get().getMeasureX().in(Meters);
            // pidOutputX = -profiledPIDController.calculate(distanceX, AutoConstants.distanceFromReef.in(Meters));

            double distanceY = translationOptional.get().getMeasureY().in(Meters);
            pidOutputY = -PIDControllerY.calculate(distanceY);
        }

        SmartDashboard.putBoolean("Y at setpoint", PIDControllerY.atSetpoint());

        double timeSinceStart = Math.abs(RobotController.getFPGATime() - timestampStart) / 1e6;

        if (isAlignedDebouncer.calculate(PIDControllerY.atSetpoint() && driveRequest.HeadingController.atSetpoint()) || isAlignedForAuto) { // || timeSinceStart > 2.5) { // && profiledPIDControllerX.atGoal()
            vision.isAlignedTimestamp = RobotController.getFPGATime();
            double distanceMeasurement = distance.getRange();
            if (autoX && timeSinceStart > 0.5) {
                isAlignedForAuto = true;
                limitTag = Optional.of(new int[] { tagID } );
                if (distanceMeasurement != -1) {
                    pidOutputX = profiledPIDControllerX.calculate(Inches.of(distanceMeasurement).in(Meters));
                } else {
                    pidOutputX = profiledPIDControllerX.calculate(2);
                }
            }
        }

        Rotation2d angleOffset = Rotation2d.kZero;

        switch (tagID) {
            case 17:
            case 11:
                angleOffset = Rotation2d.fromDegrees(-120);
                break;
            case 18:
            case 10:
                angleOffset = Rotation2d.k180deg;
                break;
            case 19:
            case 9:
                angleOffset = Rotation2d.fromDegrees(120);
                break;
            case 20:
            case 8:
                angleOffset = Rotation2d.fromDegrees(60);
                break;
            case 22:
            case 6:
                angleOffset = Rotation2d.fromDegrees(-60);
                break;
        }

        Translation2d translation2d = new Translation2d(pidOutputX, pidOutputY).rotateBy(Rotation2d.kZero.plus(angleOffset));

        driveRequest.withVelocityX(Meters.of(translation2d.getX()).per(Second));
        driveRequest.withVelocityY(Meters.of(translation2d.getY()).per(Second));
        driveRequest.withTargetDirection(Rotation2d.fromDegrees(90).plus(angleOffset));

        drivetrain.setControl(driveRequest);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    
}
