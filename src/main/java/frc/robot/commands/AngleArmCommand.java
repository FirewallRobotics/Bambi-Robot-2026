package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;

public class AngleArmCommand extends Command {
  private final IntakeArmSubsystem m_IntakeArmSubsystem;
  private boolean goingDown;

  public AngleArmCommand(IntakeArmSubsystem ia_Subsystem, boolean goingUp) {
    m_IntakeArmSubsystem = ia_Subsystem;
    this.goingDown = goingUp;
  }

  @Override
  public void initialize() {
    m_IntakeArmSubsystem.SetAngle(goingDown);
  }

  @Override
  public boolean isFinished() {
    if (m_IntakeArmSubsystem.isAtSetpoint(goingDown)) {
      return true;
    }
    return false;
  }

  // @Override
  // public void end(boolean inter){
  //  m_IntakeArmSubsystem.StopAngle();
  // }
}
