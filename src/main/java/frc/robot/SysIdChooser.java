package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class SysIdChooser {
    private final SendableChooser<SysIdRoutine> sysIdChooser = new SendableChooser<>();

    public SysIdChooser(CommandSwerveDrivetrain commandSwerveDrivetrain, 
                        ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem) {
        sysIdChooser.addOption("Swerve Translation", commandSwerveDrivetrain.m_sysIdRoutineTranslation);
        sysIdChooser.addOption("Swerve Rotation", commandSwerveDrivetrain.m_sysIdRoutineRotation);
        sysIdChooser.addOption("Swerve Steer", commandSwerveDrivetrain.m_sysIdRoutineSteer);
        sysIdChooser.addOption("Elevator", elevatorSubsystem.elevatorSysIdRoutine);
        sysIdChooser.addOption("Pivot", pivotSubsystem.pivotSysIdRoutine);

        SmartDashboard.putData("SysID Routine", sysIdChooser);
    }

    public Command sysIdDynamicForward() {
        return sysIdChooser.getSelected().dynamic(SysIdRoutine.Direction.kForward);
    }
    public Command sysIdDynamicReverse() {
        return sysIdChooser.getSelected().dynamic(SysIdRoutine.Direction.kReverse);
    }
    public Command sysIdQuasistaticForward() {
        return sysIdChooser.getSelected().quasistatic(SysIdRoutine.Direction.kForward);
    }
    public Command sysIdQuasistaticReverse() {
        return sysIdChooser.getSelected().quasistatic(SysIdRoutine.Direction.kReverse);
    }
}
