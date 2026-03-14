package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
<<<<<<< HEAD
  private ShooterSubsystem m_ShooterSubsystem;
  private double previousRPM;

  public ShootCommand(ShooterSubsystem s_Subsystem) {
    m_ShooterSubsystem = s_Subsystem;
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
      }
    }

    previousRPM = nowRPM;
  }

  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.StopShoot();
    previousRPM = 0;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
=======
  // TODO: make this; should shoot fuel
>>>>>>> 38945d14b5a7513796bbb71667b5f5196efa6d27
}
