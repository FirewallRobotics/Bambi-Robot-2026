package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class PanicShooter extends Command {

  private final ShooterSubsystem m_ShooterSubsystem;

  public PanicShooter(ShooterSubsystem m_ShooterSubsystem) {
    this.m_ShooterSubsystem = m_ShooterSubsystem;
  }

  @Override
  public void initialize() {
    m_ShooterSubsystem.panicShooter(-1000);
  }

  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.StopShoot();
  }
}
