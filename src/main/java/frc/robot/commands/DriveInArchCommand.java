package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DriveAssistanceSubsystem;

public class DriveInArchCommand extends Command {
    
    DriveAssistanceSubsystem driverassitance;
    CommandSwerveDrivetrain drivetrain;
    CommandXboxController joystick;

    public DriveInArchCommand(DriveAssistanceSubsystem driverassitance, CommandSwerveDrivetrain drivetrain, CommandXboxController joystick){
        this.driverassitance = driverassitance;
        this.drivetrain = drivetrain;
        this.joystick = joystick;
    }

    @Override
    public void execute(){
        driverassitance.driveFacingHUB(
            -joystick.getLeftY(),
            -joystick.getLeftX());
    }

    public boolean isFinished(){
        return false;
    }
}
