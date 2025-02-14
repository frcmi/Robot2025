// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Please do not initialize this more than once unless you know what you are doing, it will cause
 * problems with AdvantageKit and will cause a resource leak with the power distribution.
 */
public final class Robot extends LoggedRobot {
  private Command m_AutonomousCommand;
  private RobotContainer m_RobotContainer;

  @SuppressWarnings("resource")
  public Robot() {
    Logger.recordMetadata("Codebase", "2025-Reefscape");
    Logger.recordMetadata("Git SHA", BuildConstants.GIT_SHA);
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      if (!Constants.TelemetryConstants.disableNetworkLogging) {
        Logger.addDataReceiver(new NT4Publisher());
      }
      new PowerDistribution(1, ModuleType.kRev);
    } else if (Constants.replay) {
      setUseTiming(false);
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }
    Logger.start();
  }

  @Override
  public void robotInit() {
    m_RobotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_AutonomousCommand = m_RobotContainer.getAutonomousCommand();

    if (m_AutonomousCommand != null) {
      m_AutonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

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
