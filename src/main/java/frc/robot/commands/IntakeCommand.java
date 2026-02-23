package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;


public class IntakeCommand extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;

    public IntakeCommand(IntakeSubsystem i_Subsystem){
        m_IntakeSubsystem = i_Subsystem;
    }

    @Override
    public void execute() {
        m_IntakeSubsystem.StartIntake();
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.StopIntake();
    }


}
