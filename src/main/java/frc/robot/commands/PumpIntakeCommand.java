package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;

public class PumpIntakeCommand extends Command {

  IntakeArmSubsystem intakeArmSubsystem;
  double topsetpoint = 0.25;
  double bottomsetpoint = 0.18;
  boolean GotoTop;

  public PumpIntakeCommand(IntakeArmSubsystem intakeArmSubsystem) {
    this.intakeArmSubsystem = intakeArmSubsystem;
  }

  @Override
  public void execute() {
    if (GotoTop) {
      intakeArmSubsystem.AngleArmToSetpoint(topsetpoint);
      if (intakeArmSubsystem.isAtSetpoint(topsetpoint)) {
        GotoTop = false;
      }
    } else {
      intakeArmSubsystem.AngleArmToSetpoint(bottomsetpoint);
      if (intakeArmSubsystem.isAtSetpoint(bottomsetpoint)) {
        GotoTop = true;
      }
    }
  }
}
