
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SysIdRoutine;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.ultralogger.UltraDoubleLog;
import frc.robot.Constants.BotType;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsAlpha;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public final class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.RobotCentric autoDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry();

    private final CommandXboxController m_Controller = new CommandXboxController(0);
    private final CommandXboxController m_ScuffedController = new CommandXboxController(2);
    private final CommandXboxController m_TuningController = new CommandXboxController(4);
    private final CommandJoystick m_OperatorController = new CommandJoystick(1);

    public final CommandSwerveDrivetrain drivetrain;

    public final BotType botType = RobotDiscoverer.getRobot();

  private VisionSubsystem m_Vision;
  private LEDSubsystem m_LedSubsystem = new LEDSubsystem();
  private ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private ClawSubsystem m_ClawSubsystem = new ClawSubsystem(botType);
  private ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem(botType);
  public PivotSubsystem m_PivotSubsystem = new PivotSubsystem(botType, m_ElevatorSubsystem.elevator);

  private int algaeLevel = 0;
  private UltraDoubleLog levelLog = new UltraDoubleLog("Algae Level");

  private final SendableChooser<Command> autoChooser;
  private final SysIdChooser sysIdChooser;

  Alert onMainAlert = new Alert("Main Bot", AlertType.kInfo);
  Alert onAlphaAlert = new Alert("Alpha Bot", AlertType.kWarning);
  Alert onSimAlert = new Alert("Sim Bot", AlertType.kInfo);

  RadioLogger radioLogger = new RadioLogger();

  public RobotContainer() {
    if (botType == BotType.ALPHA_BOT) {
      drivetrain = TunerConstantsAlpha.createDrivetrain();
    } else {
      drivetrain = TunerConstants.createDrivetrain();
    }
    sysIdChooser = new SysIdChooser(drivetrain, m_ElevatorSubsystem, m_PivotSubsystem);
    
    initSubsystems();

    switch (botType) {
      case MAIN_BOT:
        onMainAlert.set(true);
        break;
      case ALPHA_BOT:
        onAlphaAlert.set(true);
        break;
      case SIM_BOT:
        onSimAlert.set(true);
        break;
    }

    autoChooser = AutoBuilder.buildAutoChooser();
    initManualAutos();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // SignalLogger.setPath("/ctre-logs/");
    SignalLogger.start();
    if (RobotBase.isReal())
      configureBindings();
    else if (RobotBase.isSimulation())
      configureSimBindings();
    m_ElevatorSubsystem.setDefaultCommand(m_ElevatorSubsystem.runSpeed(m_Controller::getLeftY));
  }
  
  private void initManualAutos() {
    autoChooser.addOption("Travel", drivetrain.applyRequest(() -> drive.withVelocityX(-1)).withTimeout(2));
  }

  private void initSubsystems() {
    m_Vision = VisionSubsystem.configure(drivetrain);
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
    ParallelCommandGroup parallelLevelCommands = null;// new ParallelCommandGroup(m_PivotSubsystem.goToAngle(algaeLevel), m_ElevatorSubsystem.goToHeight(algaeLevel));
    parallelLevelCommands.schedule();
  }

  public SwerveRequest getDriveReq() {
    double multiplier = 1;

    if (m_Controller.rightTrigger().getAsBoolean()) {
      multiplier /= 4;
    }

    if (m_ElevatorSubsystem.getElevatorHeight() > 15) {
      multiplier /= 2;
    }
    
    if (m_ElevatorSubsystem.getElevatorHeight() > 30) {
      multiplier /= 2;
    }

    return drive.withVelocityX(-m_Controller.getLeftY() * MaxSpeed * multiplier) // Drive forward with negative Y (forward)
      .withVelocityY(-m_Controller.getLeftX() * MaxSpeed * multiplier) // Drive left with negative X (left)
      .withRotationalRate(-m_Controller.getRightX() * MaxAngularRate * multiplier); // Drive counterclockwise with negative X (left)
  }

  private void configureSwerveBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(this::getDriveReq)
    );

    // m_Controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // m_Controller.b().whileTrue(drivetrain.applyRequest(() ->
    //     point.withModuleDirection(new Rotation2d(-m_Controller.getLeftY(), -m_Controller.getLeftX()))
    // ));

    // // reset the field-centric heading on left bumper press
    m_Controller.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureBindings() {
    configureSwerveBindings();
    configureTuningBindings();
    // configureScuffedBindings();
    configureOperatorBindings();
    m_LedSubsystem.setDefaultCommand(m_LedSubsystem.fauxRSL());
    
    // TODO: correct color
    new Trigger(m_ClawSubsystem.beambreak::get).negate().whileTrue(m_LedSubsystem.solidColor(new Color(0, 155, 255)));

    // TODO: add elevator and pivot setpoints to intake/shoot
    // m_Controller.rightBumper().whileTrue(m_ClawSubsystem.intakeWithBeambreak());
    // m_Controller.rightTrigger().whileTrue(m_PivotSubsystem.setAngle(Rotations.of(0.3)).until(m_PivotSubsystem::closeEnough).andThen(m_ClawSubsystem.shoot()));
    // m_Controller.leftBumper().whileTrue(m_ClimberSubsystem.runClimberup());
    // m_Controller.leftTrigger().whileTrue(m_ClimberSubsystem.runClimberdown());

    // m_Controller.povUp().onTrue(Commands.runOnce(() -> changeLevel(true)));
    // m_Controller.povDown().onTrue(Commands.runOnce(() -> changeLevel(false)));

    // m_Controller.leftBumper().whileTrue(m_PivotSubsystem.setAngle(Constants.PivotConstants.reefOneAngle));
    // m_Controller.leftTrigger().whileTrue(m_PivotSubsystem.setAngle(Constants.PivotConstants.reefTwoAngle));

    m_Controller.povDown().whileTrue(m_ElevatorSubsystem.autoHonePose().withName("Elevator Hone Command"));
  }

  public Command scuffedElevator(double rotations) {
    return Commands.runOnce(() -> m_ElevatorSubsystem.extendArm(rotations)).andThen(Commands.none().withTimeout(0.05));
  }

  public Command scuffedPivot(Angle rotations) {
    final Angle rotations2;
    if (botType == BotType.MAIN_BOT) {
      rotations2 = rotations.minus(Rotations.of(0.3 - 0.29393715734842896));
    } else {
      rotations2 = rotations;
    }

    return Commands.runOnce(() -> m_PivotSubsystem.setAngle(rotations2)).andThen(Commands.run(() -> {})).withTimeout(0.1);
  }

  public void configureScuffedBindings() {
    m_ScuffedController.y().onTrue(scuffedElevator(Constants.ElevatorConstants.bargeHeight).andThen(scuffedPivot(Constants.PivotConstants.bargeAngle)));
    m_ScuffedController.a().onTrue(scuffedElevator(Constants.ElevatorConstants.floorHeight).andThen(scuffedPivot(Constants.PivotConstants.floorAngle)));
    m_ScuffedController.x().onTrue(scuffedElevator(Constants.ElevatorConstants.stowHeight).andThen(scuffedPivot(Constants.PivotConstants.stowAngle)));
    m_ScuffedController.b().onTrue(scuffedElevator(Constants.ElevatorConstants.floorHeight).andThen(scuffedPivot(Constants.PivotConstants.processorAngle)));
    m_ScuffedController.povDown().onTrue(scuffedElevator(Constants.ElevatorConstants.reefOneHeight).andThen(scuffedPivot(Constants.PivotConstants.reefOneAngle)));
    m_ScuffedController.povUp().onTrue(scuffedElevator(Constants.ElevatorConstants.reefTwoHeight).andThen(scuffedPivot(Constants.PivotConstants.reefTwoAngle)));
    // m_ScuffedController.povLeft().onTrue(scuffedElevator(Constants.ElevatorConstants.onCoralHeight).andThen(scuffedPivot(Constants.PivotConstants.onCoralAngle)));

    m_ScuffedController.leftTrigger().whileTrue(m_ClawSubsystem.intakeWithBeambreak());
    m_ScuffedController.leftBumper().whileTrue(m_ClawSubsystem.intake());
    m_ScuffedController.rightTrigger().whileTrue(m_ClawSubsystem.shootWithBeambreak());
    m_ScuffedController.rightBumper().whileTrue(m_ClawSubsystem.shoot());
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

  private void configureOperatorBindings() {
    m_OperatorController.button(7).onTrue(scuffedElevator(Constants.ElevatorConstants.bargeHeight).andThen(scuffedPivot(Constants.PivotConstants.bargeAngle)));
    m_OperatorController.button(5).onTrue(scuffedElevator(Constants.ElevatorConstants.floorHeight).andThen(scuffedPivot(Constants.PivotConstants.floorAngle)));
    m_OperatorController.button(8).onTrue(scuffedElevator(Constants.ElevatorConstants.stowHeight).andThen(scuffedPivot(Constants.PivotConstants.stowAngle)));
    m_OperatorController.button(2).onTrue(scuffedElevator(Constants.ElevatorConstants.floorHeight).andThen(scuffedPivot(Constants.PivotConstants.processorAngle)));
    m_OperatorController.button(4).onTrue(scuffedElevator(Constants.ElevatorConstants.reefOneHeight).andThen(scuffedPivot(Constants.PivotConstants.reefOneAngle)));
    m_OperatorController.button(1).onTrue(scuffedElevator(Constants.ElevatorConstants.reefTwoHeight).andThen(scuffedPivot(Constants.PivotConstants.reefTwoAngle)));
    m_OperatorController.button(6).whileTrue(m_ClawSubsystem.intakeWithBeambreak());
    m_OperatorController.button(9).whileTrue(m_ClawSubsystem.intake());
    // m_OperatorController.button().whileTrue(m_ClawSubsystem.shootWithBeambreak());
    m_OperatorController.button(3).whileTrue(m_ClawSubsystem.shoot());
  }

  private void configureTuningBindings() {
    m_TuningController.a().whileTrue(sysIdChooser.sysIdDynamicForward());
    m_TuningController.x().whileTrue(sysIdChooser.sysIdDynamicReverse());
    m_TuningController.b().whileTrue(sysIdChooser.sysIdQuasistaticForward());
    m_TuningController.y().whileTrue(sysIdChooser.sysIdQuasistaticReverse());
  }

  boolean zeroed = false;

  public Command getAutonomousCommand() {
    Command base = Commands.none();
    if (!zeroed && Robot.isReal()) {
      base = Commands.run(() -> { zeroed = true; }).until(m_PivotSubsystem::closeEnough).andThen(m_ElevatorSubsystem.autoHonePose());
    }
    return base.andThen(autoChooser.getSelected().asProxy());
  }
}
