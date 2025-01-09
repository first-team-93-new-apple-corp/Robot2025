// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LED;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  // private final LED m_LED;

  public Robot() {
    m_robotContainer = new RobotContainer();
    PathfindingCommand.warmupCommand().schedule();
    Pathfinding.setPathfinder(new LocalADStar());
    // addPeriodic(() -> m_robotContainer.m_LED.twoColorCycle(5, Color.kGreen, Color.kBlack, 144, 25), 0.6, 0.005);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putData(CommandScheduler.getInstance());
    if (m_robotContainer.m_DriveSubsystem.getCurrentCommand() != null) {
      SmartDashboard.putString("Swerve", m_robotContainer.m_DriveSubsystem.getCurrentCommand().getName());
    } else {
      SmartDashboard.putString("Swerve", "---");
    }
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
  
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
    m_robotContainer.disableLockWheels();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
