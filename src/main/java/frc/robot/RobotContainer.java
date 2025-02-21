// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.ultralogger.UltraDoubleLog;
import frc.robot.Constants.BotType;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ClimberCostansts;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw.ClawIOTalonFXS;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Climber.ClimberIOTalonFX;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.GyroIOPigeon2;
import frc.robot.subsystems.Drive.ModuleIOTalonFX;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Pivot.PivotIOSim;
import frc.robot.subsystems.Pivot.PivotIOTalonFX;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

public final class RobotContainer {
  @SuppressWarnings("unused")
  private final SwerveRequest.SwerveDriveBrake brakeSwerve = new SwerveRequest.SwerveDriveBrake();

  private final CommandXboxController m_Controller = new CommandXboxController(0);
  private final CommandJoystick m_OperatorController = new CommandJoystick(1);
  private final CommandXboxController m_SimController = new CommandXboxController(3);

  public final Drive drivetrain =
      new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(TunerConstants.FrontLeft),
          new ModuleIOTalonFX(TunerConstants.FrontRight),
          new ModuleIOTalonFX(TunerConstants.BackLeft),
          new ModuleIOTalonFX(TunerConstants.BackRight));

  public static final BotType botType = RobotDiscoverer.getRobot();

  private VisionSubsystem m_Vision; // = new VisionSubsystem();
  private LEDSubsystem m_LedSubsystem = new LEDSubsystem();
  private ClimberSubsystem m_ClimberSubsystem =
      new ClimberSubsystem(new ClimberIOTalonFX(ClimberCostansts.climberMotorID));
  private ClawSubsystem m_ClawSubsystem =
      new ClawSubsystem(
          new ClawIOTalonFXS(ClawConstants.beambreakID, ClawConstants.motorControllerID));
  private ElevatorSubsystem m_ElevatorSubsystem;
  public PivotSubsystem m_PivotSubsystem;

  private int algaeLevel = 0;
  private UltraDoubleLog levelLog = new UltraDoubleLog("Algae Level");

  private final SendableChooser<Command> autoChooser;

  Alert onMainAlert = new Alert("Main Bot", AlertType.kInfo);
  Alert onAlphaAlert = new Alert("Alpha Bot", AlertType.kWarning);
  Alert onSimAlert = new Alert("Sim Bot", AlertType.kInfo);

  public RobotContainer() {
    if (Robot.isReal()) {
      initSubsystems();
    } else if (Robot.isSimulation()) {
      initSubsystemsSim();
    }

    switch (botType) {
      case MAIN_BOT:
        onMainAlert.set(true);
        break;
      case ALPHA_BOT:
        onAlphaAlert.set(true);
        break;
      case SIM_BOT:
      case REPLAY_BOT:
        onSimAlert.set(true);
        break;
    }

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // SignalLogger.setPath("/ctre-logs/");
    SignalLogger.start();
    configureBindings();
  }

  private void initSubsystems() {
    m_Vision = VisionSubsystem.configure(drivetrain);
    m_ElevatorSubsystem =
        new ElevatorSubsystem(
            new ElevatorIOTalonFX(
                botType,
                ElevatorConstants.leftMotorID,
                ElevatorConstants.rightMotorID,
                ElevatorConstants.upperLimitSwitchID,
                ElevatorConstants.lowerLimitSwitchID));
    m_PivotSubsystem =
        new PivotSubsystem(
            botType, new PivotIOTalonFX(PivotConstants.motorID, PivotConstants.encoderID));
    m_PivotSubsystem.addLigament(m_ElevatorSubsystem.elevator);
  }

  private void initSubsystemsSim() {
    m_ElevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
    m_PivotSubsystem = new PivotSubsystem(botType, new PivotIOSim());
    m_PivotSubsystem.addLigament(m_ElevatorSubsystem.elevator);
  }

  private void changeLevel(boolean moveUp) {
    if (moveUp) algaeLevel++;
    else algaeLevel--;

    if (algaeLevel > 4) algaeLevel = 4;
    if (algaeLevel < 0) algaeLevel = 0;

    levelLog.update((double) algaeLevel);
    ParallelCommandGroup parallelLevelCommands =
        new ParallelCommandGroup(
            m_PivotSubsystem.goToAngle(algaeLevel), m_ElevatorSubsystem.goToHeight(algaeLevel));
    parallelLevelCommands.schedule();
  }

  private void configureSwerveBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        DriveCommands.joystickDrive(
            drivetrain,
            () -> -m_Controller.getLeftY(),
            () -> -m_Controller.getLeftY(),
            () -> -m_Controller.getLeftY()));

    // m_Controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // m_Controller.b().whileTrue(drivetrain.applyRequest(() ->
    //     point.withModuleDirection(new Rotation2d(-m_Controller.getLeftY(),
    // -m_Controller.getLeftX()))
    // ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // m_Controller.back().and(m_Controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // m_Controller.back().and(m_Controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // m_Controller.start().and(m_Controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // m_Controller.start().and(m_Controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // // reset the field-centric heading on left bumper press
    // m_Controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
  }

  private void configureBindings() {
    configureSwerveBindings();
    configureOperatorBindings();
    configureTuningBindings();
    configureSimBindings();
    m_LedSubsystem.setDefaultCommand(m_LedSubsystem.fauxRSL());

    m_Controller.rightBumper().whileTrue(m_ClawSubsystem.intakeWithBeambreak());
    m_Controller.rightTrigger().whileTrue(m_ClawSubsystem.shootWithBeambreak());
    m_Controller.leftBumper().whileTrue(m_ClimberSubsystem.runClimberup());
    m_Controller.leftTrigger().whileTrue(m_ClimberSubsystem.runClimberdown());

    // m_Controller.povUp().onTrue(Commands.runOnce(() -> changeLevel(true)));
    // m_Controller.povDown().onTrue(Commands.runOnce(() -> changeLevel(false)));

    m_Controller.povLeft().whileTrue(m_PivotSubsystem.goToAngle(Rotations.of(0.066)));
    m_Controller.povRight().whileTrue(m_PivotSubsystem.goToAngle(Rotations.of(0.3)));

    m_Controller
        .povDown()
        .whileTrue(m_ElevatorSubsystem.autoHonePose().withName("Elevator Hone Command"));

    // TODO: add elevator and pivot setpoints to intake/shoot

    // when stowing the arm, use the command that goes to the floor position
    // m_Controller.a().onTrue(m_PivotSubsystem.goToFloorPosition().andThen(m_ElevatorSubsystem.goToFloorHeightCommand()));
    // m_Controller.b().onTrue(m_PivotSubsystem.goToOnCoralPosition().andThen(m_ElevatorSubsystem.goToOnCoralHeightCommand()));
    // there are two heights with the coral, don't be comfused by any of it
    // m_Controller.x().onTrue(m_PivotSubsystem.goToReefPosition().andThen(m_ElevatorSubsystem.goToReefOneHeightCommand()));
    // m_Controller.y().onTrue(m_PivotSubsystem.goToReefPosition().andThen(m_ElevatorSubsystem.goToReefTwoHeightCommand()));
    // m_Controller.povUp().onTrue(m_PivotSubsystem.goToBargePosition().andThen(m_ElevatorSubsystem.goToBargeHeightCommand()));
  }

  private void configureSimBindings() {
    m_SimController.povUp().onTrue(Commands.runOnce(() -> changeLevel(true)));
    m_SimController.povDown().onTrue(Commands.runOnce(() -> changeLevel(false)));

    // m_Controller.button(1).onTrue(m_ElevatorSubsystem.goToFloorHeightCommand().andThen(m_PivotSubsystem.goToFloorPosition()));
    // m_Controller.button(2).onTrue(m_ElevatorSubsystem.goToOnCoralHeightCommand().andThen(m_PivotSubsystem.goToOnCoralPosition()));
    // m_Controller.button(3).onTrue(m_ElevatorSubsystem.goToReefTwoHeightCommand().andThen(m_PivotSubsystem.goToReefPosition()));
    // m_Controller.button(4).onTrue(m_ElevatorSubsystem.goToBargeHeightCommand().andThen(m_PivotSubsystem.goToBargePosition()));
  }

  private void configureOperatorBindings() {
    m_OperatorController.button(1).whileTrue(m_ClawSubsystem.intakeWithBeambreak());
    m_OperatorController.button(2).whileTrue(m_ClawSubsystem.shootWithBeambreak());

    m_OperatorController.button(3).whileTrue(m_ElevatorSubsystem.goToReefOneHeightCommand());
    m_OperatorController.button(3).whileTrue(m_PivotSubsystem.goToReefOneAngle());

    m_OperatorController.button(4).whileTrue(m_ElevatorSubsystem.goToReefTwoHeightCommand());
    m_OperatorController.button(4).whileTrue(m_PivotSubsystem.goToReefTwoAngle());

    m_OperatorController.button(5).whileTrue(m_ElevatorSubsystem.goToBargeHeightCommand());
    m_OperatorController.button(5).onTrue(m_PivotSubsystem.goToBargeAngle());

    m_OperatorController.button(6).whileTrue(m_ElevatorSubsystem.goToFloorHeightCommand());
    m_OperatorController.button(6).whileTrue(m_PivotSubsystem.goToFloorAngle());

    m_OperatorController.button(7).onTrue(m_ElevatorSubsystem.goToOnCoralHeightCommand());
    m_OperatorController.button(7).onTrue(m_PivotSubsystem.goToOnCoralAngle());
  }

  private void configureTuningBindings() {
    // m_TuningController.a().whileTrue(sysIdChooser.sysIdDynamicForward());
    // m_TuningController.x().whileTrue(sysIdChooser.sysIdDynamicReverse());
    // m_TuningController.b().whileTrue(sysIdChooser.sysIdQuasistaticForward());
    // m_TuningController.y().whileTrue(sysIdChooser.sysIdQuasistaticReverse());
  }

  boolean zeroed = false;

  public Command getAutonomousCommand() {
    Command base = Commands.none();
    if (!zeroed) {
      base =
          Commands.run(
                  () -> {
                    zeroed = true;
                  })
              .until(m_PivotSubsystem::closeEnough)
              .andThen(m_ElevatorSubsystem.autoHonePose());
    }
    return base.andThen(autoChooser.getSelected());
  }
}
