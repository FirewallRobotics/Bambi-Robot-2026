package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  private ShooterSubsystem m_ShooterSubsystem;
  private AgitatorSubsystem m_AgitatorSubsystem;
  private double previousRPM;

  public ShootCommand(ShooterSubsystem s_Subsystem, AgitatorSubsystem a_AgitatorSubsystem) {
    m_ShooterSubsystem = s_Subsystem;
    m_AgitatorSubsystem = a_AgitatorSubsystem;
    previousRPM = 0;
  }

  @Override
  public void execute() {

    m_ShooterSubsystem.Shoot();

    double nowRPM = m_ShooterSubsystem.GetRPM();
    double rpmRampUp = nowRPM - previousRPM;

    if (Math.abs(rpmRampUp) < 50) {
      if ((m_ShooterSubsystem.GetWantedVelocity() - 70) < m_ShooterSubsystem.GetRPM()
          && m_ShooterSubsystem.GetRPM() < (m_ShooterSubsystem.GetWantedVelocity() + 20)) {
        m_ShooterSubsystem.KickBalls();
        m_AgitatorSubsystem.StartAgitator();
      }
    }

    previousRPM = nowRPM;
  }

  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.StopShoot();
    m_AgitatorSubsystem.StopAgitator();
    previousRPM = 0;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
