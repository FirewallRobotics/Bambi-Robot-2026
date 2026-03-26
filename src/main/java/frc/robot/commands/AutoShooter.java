package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShooter extends Command {
    //Meant to be the shooter for autonmous peroid
    private final ShooterSubsystem m_ShooterSubsystem;
    private final KickerSubsystem m_KickerSubsystem;
    private final boolean goingBack;

    public AutoShooter(
        ShooterSubsystem shooterSubsystem,
        KickerSubsystem kickerSubsystem,
        boolean goingBack) {
        m_ShooterSubsystem = shooterSubsystem;
        m_KickerSubsystem = kickerSubsystem;
        this.goingBack = goingBack;
    }

    @Override
    public void execute() {
        m_ShooterSubsystem.AutonomousShooter(goingBack);


        if ((m_ShooterSubsystem.GetAutoVelocity() - 500) < m_ShooterSubsystem.GetRPM()
            && m_ShooterSubsystem.GetRPM() < (m_ShooterSubsystem.GetAutoVelocity() + 100)) {
            m_KickerSubsystem.KickBalls();
        }
        

    }

    @Override
    public void end(boolean interrupted) {
        m_ShooterSubsystem.StopShoot();
        m_KickerSubsystem.stopKicker();
    }
}
