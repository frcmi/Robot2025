package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.revrobotics.Rev2mDistanceSensor;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.subsystems.ClawSubsystemTurbo;

public class CoralAutoBuilder {
    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private static final SwerveRequest.FieldCentricFacingAngle finalDrive = new SwerveRequest.FieldCentricFacingAngle()
      .withTargetDirection(Rotation2d.fromDegrees(-90))
      .withVelocityY(1.8)
      .withVelocityX(0.6)
      .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    private static final SwerveRequest.FieldCentricFacingAngle finalAlign = new SwerveRequest.FieldCentricFacingAngle()
      .withTargetDirection(Rotation2d.fromDegrees(90))
      .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    
    static {
        finalDrive.HeadingController.setPID(AutoConstants.Turbo.kRotationP, AutoConstants.Turbo.kRotationI, AutoConstants.Turbo.kRotationD);
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
        
        return drive
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
        return Commands.runOnce(() -> elevator.extendArm(rotations)).andThen(Commands.none().withTimeout(0.05));
    }

    public enum AutoType {
        One,
        OneAndHalf,
        Two,
    }

    public static Command build(AutoType auto, Rev2mDistanceSensor distance, CommandSwerveDrivetrain swerve, PivotSubsystem pivot, ElevatorSubsystem elevator, ClawSubsystemTurbo claw, TrigVisionSubsystem vision) {
        Command base = scuffedElevator(elevator, 4.5)
            .andThen(pivot.scuffedPivot(Rotations.of(0.075)))
            .andThen(new WaitCommand(5).until(pivot::closeEnough))
            .andThen(driveCommand(distance, swerve, 23.5, 1).until(() -> distance.getRange() < 23.5 && distance.getRange() != -1))
            .andThen(claw.runMotor(new DutyCycleOut(-0.25)).withTimeout(0.1))
            .andThen(claw.stop().withTimeout(0.1))
            .andThen(driveCommand(distance, swerve, 23, -1).until(() -> distance.getRange() > 23));
        
        Command stow = scuffedElevator(elevator, ElevatorConstants.stowHeight)
            .andThen(pivot.scuffedPivot(PivotConstants.stowAngle));

        Command half = scuffedElevator(elevator, ElevatorConstants.reefOneHeight)
            .andThen(pivot.scuffedPivot(PivotConstants.reefOneAngle))
            .andThen(new WaitCommand(1))
            .andThen(
                (
                    driveCommand(distance, swerve, 11, 1)
                    .alongWith(claw.intake())
                ).until(() -> distance.getRange() < 11 && distance.getRange() != -1).withTimeout(1.5)
            )
            .andThen(claw.intake().withTimeout(0.5))
            .andThen(
                (
                    driveCommand(distance, swerve, 25, -1)
                    .alongWith(claw.intake().withTimeout(0.25))
                ).until(() -> distance.getRange() > 25)
            )
            .andThen(claw.stop().withTimeout(0.1))
            .andThen(scuffedElevator(elevator, ElevatorConstants.stowHeight))
            .andThen(pivot.scuffedPivot(PivotConstants.stowAngle));
        
        Command shootBarge = 
            swerve.applyRequest(() -> finalDrive).until(vision::canSeeBargeTag)
            .andThen(new AlignBarge(vision, swerve, () -> 0, Inches.of(47.2), true).until(vision::isAligned))
            .andThen(scuffedElevator(elevator, ElevatorConstants.bargeHeight))
            .andThen(pivot.scuffedPivot(PivotConstants.bargeAngle))
            .andThen(new WaitCommand(3.5).until(() -> pivot.closeEnough() && elevator.closeEnough()))
            .andThen(claw.shoot().withTimeout(0.5))
            .andThen(pivot.scuffedPivot(PivotConstants.stowAngle))
            .andThen(scuffedElevator(elevator, ElevatorConstants.stowHeight))
            .andThen(new WaitCommand(1.25).alongWith(claw.stop().withTimeout(0.1)))
            .andThen(swerve.applyRequest(() -> finalAlign));

        switch (auto) {
            case One:
                return base.andThen(stow);
            case OneAndHalf:
                return base.andThen(half);
            case Two:
            default:
                return base.andThen(half).andThen(shootBarge);
        }
    }
}
