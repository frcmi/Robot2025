// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public final class RobotContainer {
  private static final double kMaxVelocity = 4.73; // m/s
  private static final double kMaxAngularVelocity = 1.5 * Math.PI; // rad/s

  private CommandXboxController m_Controller;

  private SwerveSubsystem m_Swerve;
  private VisionSubsystem m_Vision;

  private SwerveRequest.FieldCentric m_Request;

  public RobotContainer() {
    initSubsystems();
    configureBindings();
  }

  private void initSubsystems() {
    m_Swerve = SwerveSubsystem.configure();
    m_Vision = VisionSubsystem.configure(m_Swerve);
  }

  private void configureBindings() {
    m_Controller = new CommandXboxController(0);

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
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
