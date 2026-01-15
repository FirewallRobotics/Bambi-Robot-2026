package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

  private final IntakeSubsystem m_IntakeSubsystem;

  public IntakeCommand(IntakeSubsystem i_Subsystem) {
    m_IntakeSubsystem = i_Subsystem;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_IntakeSubsystem.StartIntake(1.0);
  }

  @Override
  public void end(boolean interupted) {
    m_IntakeSubsystem.StopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
