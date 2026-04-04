package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SwitchConstantOff extends Command {
    
    private boolean originalValue;

    @Override
    public void execute() {
        originalValue = ShooterSubsystem.constantShootingOn;
        ShooterSubsystem.constantShootingOn = !originalValue;
    }

    @Override
    public boolean isFinished() {
        if (!originalValue == ShooterSubsystem.constantShootingOn) {
            return true;
        }
        return false;
    }
}
