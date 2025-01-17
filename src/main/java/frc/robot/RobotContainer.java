// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public final class RobotContainer {
  private static final double kMaxVelocity = 4.73; // m/s
  private static final double kMaxAngularVelocity = 1.5 * Math.PI; // rad/s

  private CommandXboxController m_Controller = new CommandXboxController(0);

  private SwerveSubsystem m_Swerve;
  private VisionSubsystem m_Vision;
  private LEDSubsystem m_LedSubsystem = new LEDSubsystem();
  private ClawSubsystem m_ClawSubsystem = new ClawSubsystem();
  // TODO fill in parameters with the real values when possible
  private ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private PivotSubsystem m_PivotSubsystem = new PivotSubsystem(m_ElevatorSubsystem.elevator);


  private SwerveRequest.FieldCentric m_Request;

  public RobotContainer() {
    initSubsystems();
    if (RobotBase.isReal())
      configureBindings();
    else if (RobotBase.isSimulation())
      configureSimBindings();
  }

  private void initSubsystems() {
    m_Swerve = SwerveSubsystem.configure();
    m_Vision = VisionSubsystem.configure(m_Swerve);
  }

  private void configureBindings() {
    m_Request = new SwerveRequest.FieldCentric()
        .withDeadband(kMaxVelocity * 0.1)
        .withRotationalDeadband(kMaxVelocity * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    m_Swerve.setDefaultCommand(m_Swerve.applyRequest(() -> m_Request
        // stick up is -1
        // field x axis is along the length of the field
        .withVelocityX(-m_Controller.getLeftY() * kMaxVelocity)

        // stick right is +1
        // y axis is to the left
        .withVelocityY(-m_Controller.getLeftY() * kMaxVelocity)

        // stick right is +1
        // +theta is counterclockwise
        .withRotationalRate(-m_Controller.getRightX() * kMaxAngularVelocity)));

    m_LedSubsystem.setDefaultCommand(m_LedSubsystem.allianceColor());
    
    // TODO: correct color
    new Trigger(m_ClawSubsystem.beambreak::get).negate().whileTrue(m_LedSubsystem.solidColor(new Color(0, 155, 255)));

    // TODO: add elevator and pivot setpoints to intake/shoot
    m_Controller.rightBumper().whileTrue(m_ClawSubsystem.intake());
    m_Controller.rightTrigger().whileTrue(m_ClawSubsystem.shoot());
    // when stowing the arm, use the command that goes to the floor position
    m_Controller.a().onTrue(m_PivotSubsystem.goToFloorPosition().andThen(m_ElevatorSubsystem.goToFloorHeightCommand()));
    m_Controller.b().onTrue(m_PivotSubsystem.goToOnCoralPosition().andThen(m_ElevatorSubsystem.goToOnCoralHeightCommand()));
    // there are two heights with the coral, don't be comfused by any of it
    m_Controller.x().onTrue(m_PivotSubsystem.goToReefPosition().andThen(m_ElevatorSubsystem.goToReefOneHeightCommand()));
    m_Controller.y().onTrue(m_PivotSubsystem.goToReefPosition().andThen(m_ElevatorSubsystem.goToReefTwoHeightCommand()));
    m_Controller.povUp().onTrue(m_PivotSubsystem.goToBargePosition().andThen(m_ElevatorSubsystem.goToBargeHeightCommand()));
  }

  private void configureSimBindings() {
    m_Controller.button(1).onTrue(m_ElevatorSubsystem.goToOnCoralHeightCommand().andThen(m_PivotSubsystem.goToOnCoralPosition()));
    m_Controller.button(2).onTrue(m_ElevatorSubsystem.goToReefOneHeightCommand().andThen(m_PivotSubsystem.goToReefPosition()));
    m_Controller.button(3).onTrue(m_ElevatorSubsystem.goToReefTwoHeightCommand().andThen(m_PivotSubsystem.goToReefPosition()));
    m_Controller.button(4).onTrue(m_ElevatorSubsystem.goToBargeHeightCommand().andThen(m_PivotSubsystem.goToBargePosition()));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
