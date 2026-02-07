package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DriveInArchToggleCommand extends Command {

    RobotContainer robotContainer;

    public DriveInArchToggleCommand(RobotContainer robotContainer){
        this.robotContainer = robotContainer;
    }

    @Override
    public void initialize(){
        robotContainer.drivetrain.setDefaultCommand(
            new DriveInArchCommand(
                robotContainer.driveAssistanceSubsystem,
                robotContainer.drivetrain,
                robotContainer.joystick));
    }

    @Override
    public void end(boolean interrupted){
        robotContainer.DriveFieldOriented();
    }
}
