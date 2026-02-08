package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbLevel1Command extends Command {
  private final ClimberSubsystem m_ClimberSubsystem;
  private final double speed;
  public ClimbLevel1Command (ClimberSubsystem motor, double speed) {
m_ClimberSubsystem = motor;
    this.speed = speed;
  }@Override
  public void initialize() {}

  @Override
  public void execute() {
  m_ClimberSubsystem.starter(speed);
  }

  @Override
  public void end(boolean interupted) {m_ClimberSubsystem.finisher();
  
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
