package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command {

    ShooterSubsystem shooterSubsystem;

    public IntakeCommand(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize(){
        shooterSubsystem.KickBalls();
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.stopkicking();
    }
}
