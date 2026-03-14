package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AngleIntakeCommand extends Command {

  private final IntakeArmSubsystem intakeArmSubsystem;
  private boolean goingDown;

  public AngleIntakeCommand(IntakeArmSubsystem i_Subsystem, boolean goingDown) {
    intakeArmSubsystem = i_Subsystem;
    this.goingDown = goingDown;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeArmSubsystem.AngleArm(goingDown);
  }

  @Override
  public void end(boolean interupted) {
    intakeArmSubsystem.StopAngle();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
