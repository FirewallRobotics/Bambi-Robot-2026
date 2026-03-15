package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;

public class AngleArmCommand extends Command {
  private final IntakeArmSubsystem m_IntakeArmSubsystem;
  private boolean goingDown;

  public AngleArmCommand(IntakeArmSubsystem ia_Subsystem, boolean goingDown) {
    m_IntakeArmSubsystem = ia_Subsystem;
    this.goingDown = goingDown;
  }

  @Override
  public void initialize() {
    m_IntakeArmSubsystem.SetAngle(goingDown);
    ;
  }
}
