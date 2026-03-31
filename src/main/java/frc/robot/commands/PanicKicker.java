package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;

public class PanicKicker extends Command {
  private final KickerSubsystem m_KickerSubsystem;

  public PanicKicker(KickerSubsystem kickerSubsystem) {
    m_KickerSubsystem = kickerSubsystem;
  }

  @Override
  public void initialize() {

    m_KickerSubsystem.panicKickBalls();
  }

  @Override
  public void end(boolean interrupted) {
    m_KickerSubsystem.stopKicker();
  }
}
