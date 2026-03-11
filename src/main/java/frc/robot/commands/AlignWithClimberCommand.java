package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignWithClimberCommand extends Command {

  CommandSwerveDrivetrain drivetrain;

  Pose2d endPose;

  SequentialCommandGroup commandGroup;

  SwerveRequest.ApplyRobotSpeeds face = new SwerveRequest.ApplyRobotSpeeds();

  public AlignWithClimberCommand(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    endPose = Constants.DriverAssistanceConstants.endPose;
  }

  @Override
  public void initialize() {

    Pose2d waypoint;

    if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
      waypoint = new Pose2d(endPose.getX() + 1, endPose.getY(), endPose.getRotation());
    } else {
      waypoint = new Pose2d(endPose.getX() - 1, endPose.getY(), endPose.getRotation());
    }

    commandGroup =
        new SequentialCommandGroup(
            drivetrain.driveToPose(waypoint),
            new ClimbLevel1Command(), // Extend climber
            new ParallelDeadlineGroup(
                new WaitCommand(1), drivetrain.applyRequest(() -> face)), // move forward
            new UnClimbCommand()); // Retract climber

    CommandScheduler.getInstance().schedule(commandGroup);
  }

  @Override
  public boolean isFinished() {
    if (commandGroup != null) {
      return commandGroup.isFinished();
    }
    return false;
  }
}
