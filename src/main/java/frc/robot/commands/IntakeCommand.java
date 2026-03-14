package frc.robot.commands;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeCommand extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final AgitatorSubsystem m_AgitatorSubsystem;

    public IntakeCommand(IntakeSubsystem i_Subsystem, AgitatorSubsystem a_Subsystem){
        m_IntakeSubsystem = i_Subsystem;
        m_AgitatorSubsystem = a_Subsystem;
    }

    @Override
    public void execute() {
        //Logger.getGlobal().log(Level.INFO, "in command");
        m_IntakeSubsystem.StartIntake();
        m_AgitatorSubsystem.StartAgitator();
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.StopIntake();
    }


}
