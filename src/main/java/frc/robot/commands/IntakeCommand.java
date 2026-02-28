package frc.robot.commands;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.Command;


public class IntakeCommand extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final IntakeArmSubsystem m_IntakeArmSubsystem;

    public IntakeCommand(IntakeSubsystem i_Subsystem, IntakeArmSubsystem a_Subsystem){
        m_IntakeSubsystem = i_Subsystem;
        m_IntakeArmSubsystem = a_Subsystem;
    }

    @Override
    public void execute() {
        //Logger.getGlobal().log(Level.INFO, "in command");
        m_IntakeArmSubsystem.SetAngle(false);

        if (m_IntakeArmSubsystem.AtAngleNeeded(false)) {
            m_IntakeSubsystem.StartIntake();
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.StopIntake();
        m_IntakeArmSubsystem.SetAngle(true);
    }


}
