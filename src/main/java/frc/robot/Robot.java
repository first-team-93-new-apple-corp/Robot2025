// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ElevatorSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  // private final LED m_LED;

  public Robot() {
    m_robotContainer = new RobotContainer();
    // m_LED = new LED();
    // TODO remove lines related to the LEDs before competition
    // addPeriodic(() -> m_robotContainer.m_LED.twoColorCycle(5, Color.kGreen,
    // Color.kBlack, 144, 25), 0.6, 0.005);
    // m_robotContainer.syncTime();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    m_robotContainer.updateValues();
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

  // @Override
  // public void simulationPeriodic() {
  // m_robotContainer.updateSimValues();
  // m_ElevatorSubsystem.m_elevatorSim.setInput(m_ElevatorSubsystem.elevatorMotor.getMotorVoltage().getValueAsDouble()
  // * RobotController.getBatteryVoltage());
  // m_ElevatorSubsystem.m_elevatorSim.update(0.020);

  // RoboRioSim.setVInVoltage(
  // BatterySim.calculateDefaultBatteryLoadedVoltage(m_ElevatorSubsystem.m_elevatorSim.getCurrentDrawAmps()));

  // }
}
