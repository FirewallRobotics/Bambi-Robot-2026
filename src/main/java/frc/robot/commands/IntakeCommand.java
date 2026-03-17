package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  private final IntakeSubsystem m_IntakeSubsystem;
  private final AgitatorSubsystem m_AgitatorSubsystem;
  private final IntakeArmSubsystem m_IntakeArmSubsystem;

  public IntakeCommand(IntakeSubsystem i_Subsystem, AgitatorSubsystem a_Subsystem, IntakeArmSubsystem armSubsystem) {
    m_IntakeSubsystem = i_Subsystem;
    m_AgitatorSubsystem = a_Subsystem;
    m_IntakeArmSubsystem = armSubsystem;
  }

  @Override
  public void execute() {
    // Logger.getGlobal().log(Level.INFO, "in command");
    m_IntakeSubsystem.StartIntake();
    m_AgitatorSubsystem.StartAgitator();

    if (!m_IntakeArmSubsystem.isAtSetpoint(false)) {
      m_IntakeArmSubsystem.AngleArm(false);
    }
    
  }

  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.StopIntake();
  }
}
