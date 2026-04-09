package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwitchWiggleCommand extends Command {
  private boolean originalValue;

  @Override
  public void execute() {
    originalValue = SmartDashboard.getBoolean("Wiggle", originalValue);
    SmartDashboard.putBoolean("Wiggle", !originalValue);
  }

  @Override
  public boolean isFinished() {
    if (!originalValue == SmartDashboard.getBoolean("Wiggle", originalValue)) {
      return true;
    }
    return false;
  }
}
