package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command {
  private ShooterSubsystem m_ShooterSubsystem;

  public Shoot(ShooterSubsystem s_Subsystem) {
    m_ShooterSubsystem = s_Subsystem;
  }

  @Override
  public void execute() {
    m_ShooterSubsystem.Shoot();
  }

  @Override
  public void end(boolean end) {
    m_ShooterSubsystem.StopShoot();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
