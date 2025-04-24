
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClawSubsystemTurbo;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TrigVisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.GlobalVisionSubsystem;

import edu.wpi.first.units.measure.Angle;
import frc.lib.ultralogger.UltraDoubleLog;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BotType;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.CoralAutoBuilder.AutoType;
import frc.robot.commands.AlignBarge;
import frc.robot.commands.AlignReef;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsAlpha;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public final class RobotContainer {
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(AutoConstants.MaxSpeed * 0.1).withRotationalDeadband(AutoConstants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.RobotCentric autoDrive = new SwerveRequest.RobotCentric()
            .withDeadband(AutoConstants.MaxSpeed * 0.1).withRotationalDeadband(AutoConstants.MaxAngularRate * 0.1) // Add a 10% deadband
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

//   private SwerveSubsystem m_Swerve;
  private GlobalVisionSubsystem m_GlobalVision;
  private LEDSubsystem m_LedSubsystem = new LEDSubsystem();
  private ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private TrigVisionSubsystem m_TrigVision = new TrigVisionSubsystem(m_LedSubsystem);
  private ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem(botType);
  private ClawSubsystemTurbo m_ClawSubsystem = new ClawSubsystemTurbo(botType, m_ElevatorSubsystem);
  public PivotSubsystem m_PivotSubsystem = new PivotSubsystem(botType, m_ElevatorSubsystem.elevator);

  private int algaeLevel = 0;
  private UltraDoubleLog levelLog = new UltraDoubleLog("Algae Level");

  private final SendableChooser<Command> autoChooser;
  private final SysIdChooser sysIdChooser;

  Alert onMainAlert = new Alert("Main Bot", AlertType.kInfo);
  Alert onAlphaAlert = new Alert("Alpha Bot", AlertType.kWarning);
  Alert onSimAlert = new Alert("Sim Bot", AlertType.kInfo);

  RadioLogger radioLogger = new RadioLogger();

  private final Rev2mDistanceSensor distance;

  public RobotContainer(Rev2mDistanceSensor distance) {
    this.distance = distance;
    if (!TelemetryConstants.disableDatalog) {
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog());
    }
    
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

    autoChooser = new SendableChooser<Command>();
    initManualAutos();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // SignalLogger.setPath("/ctre-logs/");
    SignalLogger.start();
    if (RobotBase.isReal())
      configureBindings();
    else if (RobotBase.isSimulation())
      configureSimBindings();
  }

  public double getTravelDir() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
      return 1;
    }

    return -1;
  }
  
  private void initManualAutos() {
    autoChooser.setDefaultOption("None", Commands.none());
    drivetrain.orchestra.loadMusic("song6.chrp");
    autoChooser.addOption("Song", Commands.runOnce(() -> drivetrain.orchestra.play()));

    autoChooser.addOption("Travel", drivetrain.applyRequest(() -> drive.withVelocityY(getTravelDir())).withTimeout(2));
    // autoChooser.addOption("L1", CoralAutoBuilder.build(AutoType.One, distance, drivetrain, m_PivotSubsystem, m_ElevatorSubsystem, m_ClawSubsystem, m_TrigVision));
    // autoChooser.addOption("L1 + Intake Algae", CoralAutoBuilder.build(AutoType.OneAndHalf, distance, drivetrain, m_PivotSubsystem, m_ElevatorSubsystem, m_ClawSubsystem, m_TrigVision));
    // autoChooser.addOption("L1 + Shoot Algae", CoralAutoBuilder.build(AutoType.Two, distance, drivetrain, m_PivotSubsystem, m_ElevatorSubsystem, m_ClawSubsystem, m_TrigVision));
    autoChooser.addOption("L1", AlgaeAutoBuilder.build(AlgaeAutoBuilder.AutoType.Half, distance, drivetrain, m_PivotSubsystem, m_ElevatorSubsystem, m_ClawSubsystem, m_ClimberSubsystem, m_TrigVision));
    autoChooser.addOption("1 Algae", AlgaeAutoBuilder.build(AlgaeAutoBuilder.AutoType.One, distance, drivetrain, m_PivotSubsystem, m_ElevatorSubsystem, m_ClawSubsystem, m_ClimberSubsystem, m_TrigVision));
    autoChooser.addOption("1.5 Algae", AlgaeAutoBuilder.build(AlgaeAutoBuilder.AutoType.OneAndHalf, distance, drivetrain, m_PivotSubsystem, m_ElevatorSubsystem, m_ClawSubsystem, m_ClimberSubsystem, m_TrigVision));
    autoChooser.addOption("2 Algae", AlgaeAutoBuilder.build(AlgaeAutoBuilder.AutoType.Two, distance, drivetrain, m_PivotSubsystem, m_ElevatorSubsystem, m_ClawSubsystem, m_ClimberSubsystem, m_TrigVision));
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
    // ParallelCommandGroup parallelLevelCommands = new ParallelCommandGroup(m_PivotSubsystem.goToAngle(algaeLevel), m_ElevatorSubsystem.goToHeight(algaeLevel));
    // parallelLevelCommands.schedule();
  }

  public SwerveRequest.FieldCentric getFieldCentricDriveReq() {
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

    return drive.withVelocityX(-m_Controller.getLeftY() * AutoConstants.MaxSpeed * multiplier) // Drive forward with negative Y (forward)
      .withVelocityY(-m_Controller.getLeftX() * AutoConstants.MaxSpeed * multiplier) // Drive left with negative X (left)
      .withRotationalRate(-m_Controller.getRightX() * AutoConstants.MaxAngularRate * multiplier); // Drive counterclockwise with negative X (left)
  }

  public SwerveRequest getDriveReq() {
    if (DriverStation.isAutonomous()) {
      return brake;
    }
    return getFieldCentricDriveReq();
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
    m_Controller.rightBumper().whileTrue(  
      new AlignBarge(m_TrigVision, drivetrain, () -> { return getFieldCentricDriveReq().VelocityY; }, AutoConstants.distanceFromBarge, false)
    );
    
    m_Controller.a().whileTrue(
      new AlignReef(m_TrigVision, drivetrain, distance, () -> { return -getFieldCentricDriveReq().VelocityX; }, false, Optional.empty())
    );
    
    m_Controller.y().whileTrue(drivetrain.applyRequest(() -> brake));
    
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureBindings() {
    configureSwerveBindings();
    configureOperatorBindings();
    configureTuningBindings();
    configureScuffedBindings();
    
    // new Trigger(m_ClawSubsystem.beambreak::get).negate().whileTrue(m_LedSubsystem.solidColor(new Color(0, 155, 255)));

    // m_Controller.rightBumper().whileTrue(m_ClawSubsystem.intakeWithBeambreak());
    // m_Controller.rightTrigger().whileTrue(m_PivotSubsystem.setAngle(Rotations.of(0.3)).until(m_PivotSubsystem::closeEnough).andThen(m_ClawSubsystem.shoot()));
    // m_Controller.leftBumper().whileTrue(m_ClimberSubsystem.runClimberup());
    // m_Controller.leftTrigger().whileTrue(m_ClimberSubsystem.runClimberdown());

    // m_Controller.povUp().onTrue(Commands.runOnce(() -> changeLevel(true)));
    // m_Controller.povDown().onTrue(Commands.runOnce(() -> changeLevel(false)));

    // m_Controller.leftBumper().whileTrue(m_PivotSubsystem.setAngle(Constants.PivotConstants.reefOneAngle));
    // m_Controller.leftTrigger().whileTrue(m_PivotSubsystem.setAngle(Constants.PivotConstants.reefTwoAngle));

    m_Controller.leftTrigger().whileTrue(m_ClimberSubsystem.runClimberdown());
    m_Controller.b().whileTrue(m_ClimberSubsystem.runMotorDown(20));

    m_Controller.leftBumper().whileTrue(m_ClimberSubsystem.runClimberup());

    m_Controller.povDown().whileTrue(m_ElevatorSubsystem.autoHonePose().withName("Elevator Hone Command"));
    
    // m_Controller.rightBumper().onTrue(
    //   (
    //     (
    //       scuffedElevator(ElevatorConstantb  s.stowHeight).alongWith(scuffedPivot(Rotations.of(0.055), false))
    //     ).andThen(new WaitCommand(1))
    //   )
    //     .until(() -> m_PivotSubsystem.closeEnough())
    //     .andThen(() -> { m_PivotSubsystem.setDefaultCommand(m_PivotSubsystem.stop());})
    //     .andThen(m_PivotSubsystem.stop()));
  }

  public Command scuffedElevator(double rotations) {
    return Commands.runOnce(() -> m_ElevatorSubsystem.extendArm(rotations)).andThen(Commands.none().withTimeout(0.05));
  }

  public void configureScuffedBindings() {
    m_ScuffedController.y().onTrue(scuffedElevator(Constants.ElevatorConstants.bargeHeight).andThen(m_PivotSubsystem.scuffedPivot(Constants.PivotConstants.bargeAngle)));
    m_ScuffedController.a().onTrue(scuffedElevator(Constants.ElevatorConstants.floorHeight).andThen(m_PivotSubsystem.scuffedPivot(Constants.PivotConstants.floorAngle)));
    m_ScuffedController.x().onTrue(scuffedElevator(Constants.ElevatorConstants.stowHeight).andThen(m_PivotSubsystem.scuffedPivot(Constants.PivotConstants.stowAngle)));
    m_ScuffedController.b().onTrue(scuffedElevator(Constants.ElevatorConstants.floorHeight).andThen(m_PivotSubsystem.scuffedPivot(Constants.PivotConstants.processorAngle)));
    m_ScuffedController.povDown().onTrue(scuffedElevator(Constants.ElevatorConstants.reefOneHeight).andThen(m_PivotSubsystem.scuffedPivot(Constants.PivotConstants.reefOneAngle)));
    m_ScuffedController.povUp().onTrue(scuffedElevator(Constants.ElevatorConstants.reefTwoHeight).andThen(m_PivotSubsystem.scuffedPivot(Constants.PivotConstants.reefTwoAngle)));
    // m_ScuffedController.povLeft().onTrue(scuffedElevator(Constants.ElevatorConstants.onCoralHeight).andThen(scuffedPivot(Constants.PivotConstants.onCoralAngle)));

    m_ScuffedController.leftTrigger().whileTrue(m_ClawSubsystem.intakeWithBeambreak());
    m_ScuffedController.leftBumper().whileTrue(m_ClawSubsystem.intake());
    m_ScuffedController.rightTrigger().whileTrue(m_ClawSubsystem.shootWithBeambreak());
    m_ScuffedController.rightBumper().whileTrue(m_ClawSubsystem.shoot());

    m_ScuffedController.povRight().whileTrue(scuffedElevator(Constants.ElevatorConstants.coralIntake).andThen(m_PivotSubsystem.scuffedPivot(Constants.PivotConstants.coralIntake)));
    m_ScuffedController.povLeft().whileTrue(m_ElevatorSubsystem.autoHonePose().withName("Elevator Hone Command"));
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
    m_OperatorController.button(7).onTrue(scuffedElevator(Constants.ElevatorConstants.bargeHeight).andThen(m_PivotSubsystem.scuffedPivot(Constants.PivotConstants.bargeAngle)));
    m_OperatorController.button(5).onTrue(scuffedElevator(Constants.ElevatorConstants.floorHeight).andThen(m_PivotSubsystem.scuffedPivot(Constants.PivotConstants.floorAngle)));
    m_OperatorController.button(8).onTrue(scuffedElevator(Constants.ElevatorConstants.stowHeight).andThen(m_PivotSubsystem.scuffedPivot(Constants.PivotConstants.stowAngle)));
    m_OperatorController.button(2).onTrue(scuffedElevator(Constants.ElevatorConstants.floorHeight).andThen(m_PivotSubsystem.scuffedPivot(Constants.PivotConstants.processorAngle)));
    m_OperatorController.button(4).onTrue(scuffedElevator(Constants.ElevatorConstants.reefOneHeight).andThen(m_PivotSubsystem.scuffedPivot(Constants.PivotConstants.reefOneAngle)));
    m_OperatorController.button(1).onTrue(scuffedElevator(Constants.ElevatorConstants.reefTwoHeight).andThen(m_PivotSubsystem.scuffedPivot(Constants.PivotConstants.reefTwoAngle)));
    m_OperatorController.button(10).whileTrue(m_ClawSubsystem.intakeWithBeambreak());
    m_OperatorController.button(9).whileTrue(m_ClawSubsystem.intake());
    m_OperatorController.button(6).whileTrue(m_ClawSubsystem.runMotor(new DutyCycleOut(0.6)));
    // m_OperatorController.button().whileTrue(m_ClawSubsystem.shootWithBeambreak());

    m_OperatorController.button(3).whileTrue(m_ClawSubsystem.shoot());
  }

  private void configureTuningBindings() {
    m_TuningController.a().whileTrue(sysIdChooser.sysIdDynamicForward());
    m_TuningController.x().whileTrue(sysIdChooser.sysIdDynamicReverse());
    m_TuningController.b().whileTrue(sysIdChooser.sysIdQuasistaticForward());
    m_TuningController.y().whileTrue(sysIdChooser.sysIdQuasistaticReverse());
  }

  public Command getAutonomousCommand() {
    final double sign;
    
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      sign = -1;
    } else {
      sign = 1;
    }

    Command base = m_PivotSubsystem.scuffedPivot(Rotations.of(0.241))
      .andThen(Commands.runOnce(() -> {
        drivetrain.resetRotation(Rotation2d.fromDegrees(sign * 90));
        m_PivotSubsystem.reseedEncoder();
      }, drivetrain)); //.andThen(m_ElevatorSubsystem.autoHonePose().asProxy().raceWith(m_ClimberSubsystem.runClimberupAuto()));
    // if (Robot.isReal()) {
    //   base = base.andThen(Commands.run(() -> {}).until(m_PivotSubsystem::closeEnough)); //.andThen(m_ElevatorSubsystem.autoHonePose().asProxy());
    // }
    Command cmd = autoChooser.getSelected().asProxy();
    cmd.addRequirements(drivetrain);
    return base.andThen(cmd).withName("Full auto");
  }
}

