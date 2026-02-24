package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.Command;


public class IntakeCommand extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;

    public IntakeCommand(IntakeSubsystem i_Subsystem){
        m_IntakeSubsystem = i_Subsystem;
    }

    @Override
    public void execute() {
        //Logger.getGlobal().log(Level.INFO, "in command");
        m_IntakeSubsystem.StartIntake();
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.StopIntake();
    }


}
