package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;

public class ManualKicker extends Command {

    private final KickerSubsystem m_KickerSubsystem;

    public ManualKicker(KickerSubsystem kickerSubsystem){
        m_KickerSubsystem = kickerSubsystem;
    }

    @Override
    public void initialize() {
        m_KickerSubsystem.KickBalls();
    }

    @Override
    public void end(boolean interrupted) {
        m_KickerSubsystem.stopKicker();
    }
}
