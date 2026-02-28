package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ClimbLevel1Command extends Command {
  // TODO: make this; command that makes us climb to level 1

  CommandSwerveDrivetrain m_Drivetrain;
  ClimberSubsystem climb;

  public ClimbLevel1Command(CommandSwerveDrivetrain m_Drivetrain, ClimberSubsystem climb) {
    this.m_Drivetrain = m_Drivetrain;
    this.climb = climb;
  }

  @Override
  public void initialize() {
    climb.extendClimber();
    SwerveRequest.ApplyRobotSpeeds swervSpeeds = new ApplyRobotSpeeds();
    swervSpeeds.Speeds = new ChassisSpeeds(1, 1, 0);
    m_Drivetrain.applyRequest(() -> swervSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    climb.retractClimber();
  }

  public boolean isFinished() {
    return climb.isFinished();
  }
}
