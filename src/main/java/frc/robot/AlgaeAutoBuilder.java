package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TrigVisionSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.AlignBarge;
import frc.robot.commands.AlignReef;
import frc.robot.subsystems.ClawSubsystemTurbo;

public class AlgaeAutoBuilder {
    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    private static final SwerveRequest.FieldCentric driveTwoElectricBoogaloo = new SwerveRequest.FieldCentric().withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    private static final SwerveRequest.FieldCentricFacingAngle approachSecondTag = new SwerveRequest.FieldCentricFacingAngle().withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective).withTargetDirection(Rotation2d.fromDegrees(150)).withVelocityX(-1.8).withVelocityY(0.7);
    private static final SwerveRequest.RobotCentric leaveSecondTag = new SwerveRequest.RobotCentric().withVelocityY(-5);

    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private static final SwerveRequest.FieldCentricFacingAngle finalDrive = new SwerveRequest.FieldCentricFacingAngle()
      .withTargetDirection(Rotation2d.fromDegrees(-90))
      .withVelocityY(3)
      .withVelocityX(0.3)
      .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    private static final SwerveRequest.FieldCentricFacingAngle finalAlign = new SwerveRequest.FieldCentricFacingAngle()
      .withTargetDirection(Rotation2d.fromDegrees(90))
      .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    
    

    private static final SwerveRequest.FieldCentricFacingAngle driveReq = new SwerveRequest.FieldCentricFacingAngle().withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective).withTargetDirection(Rotation2d.fromDegrees(90));
    
    static {
        finalDrive.HeadingController.setPID(AutoConstants.Turbo.kRotationP, AutoConstants.Turbo.kRotationI, AutoConstants.Turbo.kRotationD);
        approachSecondTag.HeadingController.setPID(AutoConstants.Turbo.kRotationP, AutoConstants.Turbo.kRotationI, AutoConstants.Turbo.kRotationD);
        driveReq.HeadingController.setPID(AutoConstants.Turbo.kRotationP, AutoConstants.Turbo.kRotationI, AutoConstants.Turbo.kRotationD);
    }
    
    public static SwerveRequest getRequest(Rev2mDistanceSensor distance, double distanceTarget, double dir, Translation2d movement) {
        if (distance.getRange() != -1 && dir != 0) {
            if ((dir > 0 && distance.getRange() < distanceTarget) || (dir < 0 && distance.getRange() > distanceTarget)) {
                return brake;
            }
        }

        if (dir == 0) {
            dir = 1;
        }
        
        return driveReq
            .withVelocityY(dir * movement.getY())
            .withVelocityX(dir * movement.getX());
    }

    public static Command driveCommand(Rev2mDistanceSensor distance, CommandSwerveDrivetrain swerve, double distanceTarget, double dir) {
        return driveCommand(distance, swerve, distanceTarget, dir, new Translation2d(-0.8, 0));
    }

    public static Command driveCommand(Rev2mDistanceSensor distance, CommandSwerveDrivetrain swerve, double distanceTarget, double dir, Translation2d movement) {
        return swerve.applyRequest(() -> getRequest(distance, distanceTarget, dir, movement));
    }

    public static Command scuffedElevator(ElevatorSubsystem elevator, double rotations) {
        return Commands.runOnce(() -> elevator.extendArm(rotations));
    }

    public enum AutoType {
        Half,
        One,
        OneAndHalf,
        Two,
    }

    private static final SwerveRequest.FieldCentricFacingAngle firstAlign = new SwerveRequest.FieldCentricFacingAngle().withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective).withTargetDirection(Rotation2d.fromDegrees(90)).withVelocityY(0);
    static {
        firstAlign.HeadingController.setPID(AutoConstants.Turbo.kRotationP, AutoConstants.Turbo.kRotationI, AutoConstants.Turbo.kRotationD);
    }

    private static final ProfiledPIDController profiledPIDControllerX = new ProfiledPIDController(AutoConstants.Turbo.kTranslationXP, AutoConstants.Turbo.kTranslationXI, AutoConstants.Turbo.kTranslationXD, new TrapezoidProfile.Constraints(10, 1));

    public static Command firstAlign(CommandSwerveDrivetrain swerve, Rev2mDistanceSensor distance, Distance goal) {
        return Commands.runOnce(() -> { profiledPIDControllerX.setGoal(goal.in(Meters)); } ) //  profiledPIDControllerX.reset(2);
            .andThen(swerve.applyRequest(() -> {
                double distanceMeasurement = distance.getRange();
                double outputX = 0;
                if (distanceMeasurement != -1) {
                    outputX = profiledPIDControllerX.calculate(Inches.of(distanceMeasurement).in(Meters));
                } else {
                    outputX = profiledPIDControllerX.calculate(2);
                }
                return firstAlign.withVelocityX(outputX);
            })).until(profiledPIDControllerX::atGoal);
            
    }

    public static Command build(AutoType auto, Rev2mDistanceSensor distance, CommandSwerveDrivetrain swerve, PivotSubsystem pivot, ElevatorSubsystem elevator, ClawSubsystemTurbo claw, TrigVisionSubsystem vision) {
        Command stow = scuffedElevator(elevator, ElevatorConstants.stowHeight)
            .andThen(pivot.scuffedPivot(PivotConstants.stowAngle));

        Command baseCoral = Commands.runOnce(() -> SmartDashboard.putBoolean("Auto Running", true))
            .andThen(firstAlign(swerve, distance, AutoConstants.distanceFromReef)
                .alongWith(scuffedElevator(elevator, 4.5).andThen(pivot.scuffedPivot(Rotations.of(0.075))))
            )
            .andThen(claw.runMotor(new DutyCycleOut(-0.5)).withTimeout(0.2).andThen(claw.stop().withTimeout(0.1)))
            .andThen(
                (new WaitCommand(0.5)
                    .andThen(
                        scuffedElevator(elevator,ElevatorConstants.reefOneHeight)
                        .andThen(pivot.scuffedPivot(PivotConstants.reefOneAngle))
                    )
                    .alongWith(driveCommand(distance, swerve, 17, -1.5).until(() -> distance.getRange() > 17)))
            )
            .andThen(firstAlign(swerve, distance, AutoConstants.distanceFromReef)
                .alongWith(claw.intake()).until(() -> !claw.beambreak.get())
            )
            .andThen(stow.asProxy());
        

        Command base = Commands.runOnce(() -> SmartDashboard.putBoolean("Auto Running", true))
            .andThen(firstAlign(swerve, distance, AutoConstants.distanceFromReef)
                .alongWith(scuffedElevator(elevator, ElevatorConstants.reefOneHeight).andThen(pivot.scuffedPivot(PivotConstants.reefOneAngle)))
            .alongWith(claw.intake()).until(() -> !claw.beambreak.get()))
            .andThen(stow);
        
        Command shootBarge =
            (
                swerve.applyRequest(() -> driveTwoElectricBoogaloo.withVelocityX(4.0)).until(vision::canSeeBargeTag)
            ).withTimeout(0.25).andThen(
            swerve.applyRequest(() -> finalDrive).until(vision::canSeeBargeTag))
            .andThen(swerve.applyRequest(() -> driveTwoElectricBoogaloo.withVelocityX(0.7).withVelocityY(2.8)).withTimeout(0.2))
            .andThen(new AlignBarge(vision, swerve, () -> 0).until(vision::isAligned))
            .andThen(scuffedElevator(elevator, ElevatorConstants.bargeHeight))
            .andThen(pivot.scuffedPivot(PivotConstants.bargeAngle))
            .andThen(new WaitCommand(3.0).until(() -> pivot.closeEnough() && elevator.closeEnough(3.0)))
            .andThen(claw.shoot().withTimeout(0.25).andThen(claw.stop().withTimeout(0.1)))
            .andThen(pivot.scuffedPivot(PivotConstants.stowAngle))
            .andThen(scuffedElevator(elevator, ElevatorConstants.stowHeight));
            // .andThen(new WaitCommand(1.25).alongWith(claw.stop().withTimeout(0.1)))
            // .andThen(swerve.applyRequest(() -> finalAlign));

        Command half = new WaitCommand(0.8).andThen(swerve.applyRequest(() -> approachSecondTag).until(() -> vision.getReefTagID().orElseGet(() -> -1L) == 20 ));

        Command base2 = (Commands.runOnce(() -> SmartDashboard.putBoolean("Auto Running", true))
            .andThen(new AlignReef(vision, swerve, distance, () -> 0, true, Optional.of(20)))
            .alongWith(scuffedElevator(elevator, ElevatorConstants.reefTwoHeight).andThen(pivot.scuffedPivot(PivotConstants.reefTwoAngle)))
            .alongWith(claw.intake()).until(() -> !claw.beambreak.get()));

        Command shootCommand2 = (scuffedElevator(elevator, ElevatorConstants.stowHeight)
                .andThen(pivot.scuffedPivot(PivotConstants.stowAngle))
            )
            .andThen(swerve.applyRequest(() -> leaveSecondTag).withTimeout(0.2))
            .andThen(swerve.applyRequest(() -> finalDrive
                .withVelocityY(0.7)
                .withVelocityX(2.5))).until(vision::canSeeBargeTag)
            // barge shot
            .andThen(new AlignBarge(vision, swerve, () -> 0).until(vision::isAligned))
                .andThen(scuffedElevator(elevator, ElevatorConstants.bargeHeight))
                .andThen(pivot.scuffedPivot(PivotConstants.bargeAngle))
                .andThen(new WaitCommand(3.0).until(() -> pivot.closeEnough() && elevator.closeEnough(2.5)))
                .andThen(claw.shoot().withTimeout(0.25).andThen(claw.stop().withTimeout(0.1)))
                .andThen(pivot.scuffedPivot(PivotConstants.stowAngle))
                .andThen(scuffedElevator(elevator, ElevatorConstants.stowHeight))
                .andThen(new WaitCommand(0.5))
                .andThen(swerve.applyRequest(() -> drive.withVelocityX(-2).withVelocityY(0)).withTimeout(0.7));

        Command end = (stow.asProxy()).andThen(Commands.runOnce(() -> SmartDashboard.putBoolean("Auto Running", false)));

        switch (auto) {
            case Half:
                return baseCoral.asProxy();
            case One:
                return baseCoral.andThen(shootBarge).andThen(swerve.applyRequest(() -> drive.withVelocityX(-2).withVelocityY(0)).withTimeout(0.7)).andThen(end.asProxy()); //.andThen(stow);
            case OneAndHalf:
                return baseCoral.andThen(shootBarge).andThen(half).andThen(base2).andThen(end.asProxy()); //.andThen(half);
            case Two:
            default:
                return base.andThen(shootBarge).andThen(half).andThen(base2).andThen(shootCommand2.asProxy()).andThen(end.asProxy());
        }
    }
}
