package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AngleAndRunIntakeCommand extends Command {

  private final IntakeArmSubsystem m_IntakeArmSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;
  private final AgitatorSubsystem m_AgitatorSubsystem;

  public AngleAndRunIntakeCommand(
      IntakeArmSubsystem ia_Subsystem, IntakeSubsystem i_Subsystem, AgitatorSubsystem a_Subsystem) {
    m_IntakeArmSubsystem = ia_Subsystem;
    m_IntakeSubsystem = i_Subsystem;
    m_AgitatorSubsystem = a_Subsystem;
  }

  @Override
  public void initialize() {
    m_IntakeArmSubsystem.SetAngle(false);
  }

  @Override
  public void execute() {
    if (m_IntakeArmSubsystem.isAtSetpoint(false)) {
      m_IntakeSubsystem.StartIntake();
      m_AgitatorSubsystem.StartAgitator();
    }
  }

  @Override
  public void end(boolean inter) {
    m_IntakeSubsystem.StopIntake();
    m_AgitatorSubsystem.StopAgitator();
    m_IntakeArmSubsystem.SetAngle(true);
  }
}
