package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DriveAssistanceSubsystem;

public class DriveInArchCommand extends Command {

  DriveAssistanceSubsystem driverassitance;
  CommandSwerveDrivetrain drivetrain;
  CommandXboxController joystick;

  public DriveInArchCommand(
      RobotContainer robotContainer) {
    this.driverassitance = robotContainer.driveAssistanceSubsystem;
    this.drivetrain = robotContainer.drivetrain;
    this.joystick = robotContainer.joystick;
  }

  @Override
  public void execute() {
    driverassitance.driveFacingHUB(-joystick.getLeftY(), -joystick.getLeftX());
  }

  public boolean isFinished() {
    return false;
  }
}
