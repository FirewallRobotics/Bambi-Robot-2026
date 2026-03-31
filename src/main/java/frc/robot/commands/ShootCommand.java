package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  private ShooterSubsystem m_ShooterSubsystem;
  private KickerSubsystem m_KickerSubsystem;
  private double previousRPM;
  private final boolean manualShoot;

  public ShootCommand(
      ShooterSubsystem s_Subsystem, KickerSubsystem kickerSubsystem, boolean manualShoot) {
    m_ShooterSubsystem = s_Subsystem;
    m_KickerSubsystem = kickerSubsystem;
    previousRPM = 0;
    this.manualShoot = manualShoot;
  }

  @Override
  public void execute() {

    m_ShooterSubsystem.Shoot(manualShoot);

    double nowRPM = m_ShooterSubsystem.GetRPM();
    double rpmRampUp = nowRPM - previousRPM;

    if (Math.abs(rpmRampUp) < 50) {
      if ((m_ShooterSubsystem.GetWantedVelocity(manualShoot) - 70) < m_ShooterSubsystem.GetRPM()
          && m_ShooterSubsystem.GetRPM()
              < (m_ShooterSubsystem.GetWantedVelocity(manualShoot) + 20)) {
        m_KickerSubsystem.KickBalls();
      }
    }

    previousRPM = nowRPM;
  }

  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.StopShoot();
    m_KickerSubsystem.stopKicker();
    previousRPM = 0;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
