// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotController;

public final class Robot extends TimedRobot {
  private Command m_AutonomousCommand;
  private RobotContainer m_RobotContainer;
  private Rev2mDistanceSensor distance = new Rev2mDistanceSensor(Port.kMXP, Unit.kInches, RangeProfile.kHighAccuracy);

  @Override
  public void robotInit() {
    distance.setAutomaticMode(true);
    m_RobotContainer = new RobotContainer(distance);
    m_RobotContainer.drivetrain.orchestra.loadMusic("song.chrp");
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Rev Distance", distance.getRange());
    SmartDashboard.putNumber("Rev Distance Reef ", Inches.of(distance.getRange()).in(Meters));

    SmartDashboard.putBoolean("3.3V rail enabled", RobotController.getEnabled3V3());
    SmartDashboard.putBoolean("5V rail enabled", RobotController.getEnabled5V());
    SmartDashboard.putBoolean("6V rail enabled", RobotController.getEnabled6V());
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_RobotContainer.drivetrain.orchestra.pause();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if (m_AutonomousCommand != null) {
      m_AutonomousCommand.cancel();
    }
    
    m_AutonomousCommand = m_RobotContainer.getAutonomousCommand();

    if (m_AutonomousCommand != null) {
      m_AutonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    m_RobotContainer.drivetrain.orchestra.pause();
  }

  @Override
  public void teleopInit() {
    if (m_AutonomousCommand != null) {
      m_AutonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
