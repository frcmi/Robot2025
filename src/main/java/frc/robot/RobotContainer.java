// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TrigVisionSubsystem;
import edu.wpi.first.units.Units;
import frc.lib.ultralogger.UltraDoubleLog;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public final class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_Controller = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private TrigVisionSubsystem m_Vision = new TrigVisionSubsystem();
  private LEDSubsystem m_LedSubsystem = new LEDSubsystem();
  private ClawSubsystem m_ClawSubsystem = new ClawSubsystem();
  private ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private PivotSubsystem m_PivotSubsystem = new PivotSubsystem(m_ElevatorSubsystem.elevator);

  private int algaeLevel = 0;
  private UltraDoubleLog levelLog = new UltraDoubleLog("Algae Level");

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    initSubsystems();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    if (RobotBase.isReal())
      configureBindings();
    else if (RobotBase.isSimulation())
      configureSimBindings();
  }

  private void initSubsystems() {
  }

  private void changeLevel(boolean moveUp) {
    if(moveUp)
      algaeLevel++;
    else
      algaeLevel--;
    
    if(algaeLevel > 4)
      algaeLevel = 4;
    if(algaeLevel < 0)
      algaeLevel = 0;

    levelLog.update((double)algaeLevel);
    ParallelCommandGroup parallelLevelCommands = new ParallelCommandGroup(m_PivotSubsystem.goToAngle(algaeLevel), m_ElevatorSubsystem.goToHeight(algaeLevel));
    parallelLevelCommands.schedule();
  }

  private void configureSwerveBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-m_Controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-m_Controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-m_Controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
    );

    // m_Controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // m_Controller.b().whileTrue(drivetrain.applyRequest(() ->
    //     point.withModuleDirection(new Rotation2d(-m_Controller.getLeftY(), -m_Controller.getLeftX()))
    // ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // m_Controller.back().and(m_Controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // m_Controller.back().and(m_Controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // m_Controller.start().and(m_Controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // m_Controller.start().and(m_Controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // // reset the field-centric heading on left bumper press
    // m_Controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureBindings() {
    configureSwerveBindings();
    m_LedSubsystem.setDefaultCommand(m_LedSubsystem.fauxRSL());
    
    // TODO: correct color
    new Trigger(m_ClawSubsystem.beambreak::get).negate().whileTrue(m_LedSubsystem.solidColor(new Color(0, 155, 255)));

    // TODO: add elevator and pivot setpoints to intake/shoot
    m_Controller.rightBumper().whileTrue(m_ClawSubsystem.intake());
    m_Controller.rightTrigger().whileTrue(m_ClawSubsystem.shoot());

    m_Controller.povUp().onTrue(Commands.runOnce(() -> changeLevel(true)));
    m_Controller.povDown().onTrue(Commands.runOnce(() -> changeLevel(false)));

    // when stowing the arm, use the command that goes to the floor position
    // m_Controller.a().onTrue(m_PivotSubsystem.goToFloorPosition().andThen(m_ElevatorSubsystem.goToFloorHeightCommand()));
    // m_Controller.b().onTrue(m_PivotSubsystem.goToOnCoralPosition().andThen(m_ElevatorSubsystem.goToOnCoralHeightCommand()));
    // there are two heights with the coral, don't be comfused by any of it
    // m_Controller.x().onTrue(m_PivotSubsystem.goToReefPosition().andThen(m_ElevatorSubsystem.goToReefOneHeightCommand()));
    // m_Controller.y().onTrue(m_PivotSubsystem.goToReefPosition().andThen(m_ElevatorSubsystem.goToReefTwoHeightCommand()));
    // m_Controller.povUp().onTrue(m_PivotSubsystem.goToBargePosition().andThen(m_ElevatorSubsystem.goToBargeHeightCommand()));
  }

  private void configureSimBindings() {
    configureSwerveBindings();
    drivetrain.resetPose(new Pose2d(3, 3, new Rotation2d()));

    m_Controller.povUp().onTrue(Commands.runOnce(() -> changeLevel(true)));
    m_Controller.povDown().onTrue(Commands.runOnce(() -> changeLevel(false)));

    // m_Controller.button(1).onTrue(m_ElevatorSubsystem.goToFloorHeightCommand().andThen(m_PivotSubsystem.goToFloorPosition()));
    // m_Controller.button(2).onTrue(m_ElevatorSubsystem.goToOnCoralHeightCommand().andThen(m_PivotSubsystem.goToOnCoralPosition()));
    // m_Controller.button(3).onTrue(m_ElevatorSubsystem.goToReefTwoHeightCommand().andThen(m_PivotSubsystem.goToReefPosition()));
    // m_Controller.button(4).onTrue(m_ElevatorSubsystem.goToBargeHeightCommand().andThen(m_PivotSubsystem.goToBargePosition()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
