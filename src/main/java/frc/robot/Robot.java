// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ShooterSubsystem;

public class Robot extends TimedRobot {

  /** The command that is run during the auto period */
  private Command m_autonomousCommand;

  /** The single universal instance of robot container */
  private final RobotContainer m_robotContainer;

  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay =
      new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  // Instantiate the robot
  public Robot() {

    // start data logging
    DataLogManager.start();

    // port forward the second Limelight
    PortForwarder.add(5800, "172.29.0.1", 5800);
    PortForwarder.add(5801, "172.29.0.1", 5801);
    PortForwarder.add(5802, "172.29.0.1", 5802);
    PortForwarder.add(5803, "172.29.0.1", 5803);
    PortForwarder.add(5804, "172.29.0.1", 5804);
    PortForwarder.add(5805, "172.29.0.1", 5805);
    PortForwarder.add(5806, "172.29.0.1", 5806);
    PortForwarder.add(5807, "172.29.0.1", 5807);
    PortForwarder.add(5808, "172.29.0.1", 5808);
    PortForwarder.add(5809, "172.29.0.1", 5809);

    // Create robot container
    m_robotContainer = new RobotContainer();

    // starts the datalogging for the drivetrain
    m_robotContainer.getLogger().initSwervePublisher(m_robotContainer);
  }

  @Override
  public void robotPeriodic() {

    // periodically log joystick and timing data
    m_timeAndJoystickReplay.update();

    // run any commands in line to be executed
    CommandScheduler.getInstance().run();

    // run robotContainers periodic
    m_robotContainer.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // get the auto command to be run
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    ShooterSubsystem.constantShootingOn = false;
    // if that command exists schedule it to be executed
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // if the auto command exists and teleop starts then cancel it
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    // during test mode reset everytime we enable
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
