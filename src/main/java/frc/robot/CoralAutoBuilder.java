package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.ClawSubsystemTurbo;

public class CoralAutoBuilder {
    private static final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    
    public static SwerveRequest getRequest(Rev2mDistanceSensor distance, double distanceTarget, double dir) {
        if (distance.getRange() != -1) {
            if ((dir > 0 && distance.getRange() < distanceTarget) || (dir < 0 && distance.getRange() > distanceTarget)) {
                return brake;
            }
        }
        
        return drive.withVelocityY(dir * 0.5);
    }

    public static Command driveCommand(Rev2mDistanceSensor distance, CommandSwerveDrivetrain swerve, double distanceTarget, double dir) {
        return swerve.applyRequest(() -> getRequest(distance, distanceTarget, dir));
    }

    public static Command scuffedElevator(ElevatorSubsystem elevator, double rotations) {
        return Commands.runOnce(() -> elevator.extendArm(rotations)).andThen(Commands.none().withTimeout(0.05));
    }

    public static Command build(Rev2mDistanceSensor distance, CommandSwerveDrivetrain swerve, PivotSubsystem pivot, ElevatorSubsystem elevator, ClawSubsystemTurbo claw) {
        return (elevator.autoHonePose().asProxy())
            .andThen(pivot.scuffedPivot(Rotations.of(0.075), true))
            .andThen(new WaitCommand(0.1).andThen(Commands.run(() -> {}).until(pivot::closeEnough)))
            .andThen(driveCommand(distance, swerve, 20.5, 1).until(() -> distance.getRange() < 20.5 && distance.getRange() != -1))
            .andThen(claw.intake().withTimeout(0.1))
            .andThen(claw.stop().withTimeout(0.1))
            .andThen(pivot.scuffedPivot(PivotConstants.stowAngle, true).andThen(new WaitCommand(1)))
            .andThen(driveCommand(distance, swerve, 17, -1).until(() -> distance.getRange() > 17))
            .andThen(scuffedElevator(elevator, ElevatorConstants.reefOneHeight))
            .andThen(pivot.scuffedPivot(PivotConstants.reefTwoAngle, true))
            .andThen(new WaitCommand(2))
            .andThen(driveCommand(distance, swerve, 11, 1).until(() -> distance.getRange() < 11 && distance.getRange() != -1).alongWith(claw.intakeWithBeambreak()));
    }
}
