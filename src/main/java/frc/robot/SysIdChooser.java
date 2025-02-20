package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CommandSupplier;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.SysIdRoutine;

public class SysIdChooser {
    private final SendableChooser<SysIdRoutine> sysIdChooser = new SendableChooser<>();
    
    private CommandSupplier dynamicForward = new CommandSupplier();
    private CommandSupplier dynamicReverse = new CommandSupplier();
    private CommandSupplier quasistaticForward = new CommandSupplier();
    private CommandSupplier quasistaticReverse = new CommandSupplier();

    public SysIdChooser(CommandSwerveDrivetrain commandSwerveDrivetrain, 
                        ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem) {
        sysIdChooser.addOption("Swerve Translation", commandSwerveDrivetrain.m_sysIdRoutineTranslation);
        sysIdChooser.addOption("Swerve Rotation", commandSwerveDrivetrain.m_sysIdRoutineRotation);
        sysIdChooser.addOption("Swerve Steer", commandSwerveDrivetrain.m_sysIdRoutineSteer);
        sysIdChooser.addOption("Elevator", elevatorSubsystem.elevatorSysIdRoutine);
        sysIdChooser.addOption("Pivot", pivotSubsystem.pivotSysIdRoutine);

        sysIdChooser.onChange(this::updateCommands);

        SmartDashboard.putData("SysID Routine", sysIdChooser);
    }

    public void updateCommands(SysIdRoutine routine) {
        dynamicForward.setCommand(sysIdChooser.getSelected().dynamic(SysIdRoutine.Direction.kForward));
        dynamicReverse.setCommand(sysIdChooser.getSelected().dynamic(SysIdRoutine.Direction.kReverse));
        quasistaticForward.setCommand(sysIdChooser.getSelected().quasistatic(SysIdRoutine.Direction.kForward));
        quasistaticReverse.setCommand(sysIdChooser.getSelected().quasistatic(SysIdRoutine.Direction.kReverse));
    } 

    public CommandSupplier sysIdDynamicForward() {
        return dynamicForward;
    }
    public CommandSupplier sysIdDynamicReverse() {
        return dynamicReverse;
    }
    public CommandSupplier sysIdQuasistaticForward() {
        return quasistaticForward;
    }
    public CommandSupplier sysIdQuasistaticReverse() {
        return quasistaticReverse;
    }
}
