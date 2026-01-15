package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  private ShooterSubsystem m_ShooterSubsystem;

  public ShootCommand(ShooterSubsystem s_Subsystem) {
    m_ShooterSubsystem = s_Subsystem;
  }

  @Override
  public void execute() {
    m_ShooterSubsystem.Shoot(1.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.StopShoot();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
