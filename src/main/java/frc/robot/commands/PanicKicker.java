package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;

public class PanicKicker extends Command {
  private final KickerSubsystem m_KickerSubsystem;
  private final boolean isGoingUp;

  public PanicKicker(KickerSubsystem kickerSubsystem, boolean isGoingUp) {
    m_KickerSubsystem = kickerSubsystem;
    this.isGoingUp = isGoingUp;
  }

  @Override
  public void initialize() {
    if (isGoingUp) {
      m_KickerSubsystem.kickBalls(1000);
    } else{
      m_KickerSubsystem.panicKickBalls();
    }
    
  }

  @Override
  public void end(boolean interrupted) {
    m_KickerSubsystem.stopKicker();
  }
}
