package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AngleIntakeCommand extends Command {

  private final IntakeSubsystem m_IntakeSubsystem;
  private Double sppd;

  public AngleIntakeCommand(IntakeSubsystem i_Subsystem, Double speed) {
    m_IntakeSubsystem = i_Subsystem;
    sppd = speed;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_IntakeSubsystem.angleArm(sppd);
  }

  @Override
  public void end(boolean interupted) {
    m_IntakeSubsystem.angleArm(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
