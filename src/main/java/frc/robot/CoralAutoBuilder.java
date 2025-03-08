package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.ClawSubsystemTurbo;

public class CoralAutoBuilder {
    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private static final SwerveRequest.FieldCentricFacingAngle finalDrive = new SwerveRequest.FieldCentricFacingAngle()
      .withTargetDirection(new Rotation2d(Math.PI))
      .withVelocityX(1.5)
      .withVelocityY(-0.8)
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    
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
        return driveCommand(distance, swerve, distanceTarget, dir, new Translation2d(0, 0.8));
    }

    public static Command driveCommand(Rev2mDistanceSensor distance, CommandSwerveDrivetrain swerve, double distanceTarget, double dir, Translation2d movement) {
        return swerve.applyRequest(() -> getRequest(distance, distanceTarget, dir, movement));
    }

    public static Command scuffedElevator(ElevatorSubsystem elevator, double rotations) {
        return Commands.runOnce(() -> elevator.extendArm(rotations)).andThen(Commands.none().withTimeout(0.05));
    }

    public static Command build(Rev2mDistanceSensor distance, CommandSwerveDrivetrain swerve, PivotSubsystem pivot, ElevatorSubsystem elevator, ClawSubsystemTurbo claw) {
        return (elevator.autoHonePose().asProxy())
            .andThen(scuffedElevator(elevator, 3))
            .andThen(pivot.scuffedPivot(Rotations.of(0.075), true))
            .andThen(new WaitCommand(0.1).andThen(Commands.run(() -> {}).until(pivot::closeEnough)))
            .andThen(driveCommand(distance, swerve, 23.5, 1).until(() -> distance.getRange() < 23.5 && distance.getRange() != -1))
            .andThen(claw.runMotor(new DutyCycleOut(-0.25)).withTimeout(0.1))
            .andThen(claw.stop().withTimeout(0.1))
            .andThen(driveCommand(distance, swerve, 23, -1).until(() -> distance.getRange() > 23))
            // .andThen(pivot.scuffedPivot(PivotConstants.stowAngle, true).andThen(new WaitCommand(1)))
            .andThen(scuffedElevator(elevator, ElevatorConstants.reefOneHeight))
            .andThen(pivot.scuffedPivot(PivotConstants.reefTwoAngle, true))
            .andThen(new WaitCommand(1))
            .andThen(
                (
                    driveCommand(distance, swerve, 15, 1)
                    .alongWith(claw.intakeWithBeambreak()
                ).until(() -> distance.getRange() < 15 && distance.getRange() != -1)
            )
            .andThen(claw.intake().withTimeout(1))
            .andThen(
                (
                    driveCommand(distance, swerve, 25, -1)
                    .alongWith(claw.intake().withTimeout(1))
                ).until(() -> distance.getRange() > 25)
            )
            .andThen(claw.stop().withTimeout(0.1))
            .andThen(scuffedElevator(elevator, ElevatorConstants.stowHeight))
            .andThen(pivot.scuffedPivot(PivotConstants.stowAngle, true))
            .andThen(swerve.applyRequest(() -> finalDrive)).withTimeout(4.5));
    }
}
