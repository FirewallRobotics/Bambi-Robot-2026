package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.logging.Level;
import java.util.logging.Logger;

public class MoveToStaticPositionCommand extends Command {

  PathPlannerPath pathLeft;
  PathPlannerPath pathRight;
  CommandSwerveDrivetrain drivetrain;
  Command pathCommand;

  public MoveToStaticPositionCommand(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  public Pose2d Invert(Pose2d pose) {
    Logger.getGlobal().log(Level.INFO, "Inverting path");
    RotationTarget target = new RotationTarget(0, pose.getRotation());
    PathPoint temp = new PathPoint(pose.getTranslation(), target).flip();
    return new Pose2d(temp.position, temp.rotationTarget.rotation());
  }

  @Override
  public void initialize() {
    if (DriverStation.getAlliance().isEmpty()) {
      Logger.getGlobal().log(Level.WARNING, "Cannot move without an alliance");
    }

    Logger.getGlobal()
        .log(
            Level.INFO,
            "Target distance left: "
                + (Constants.DriverAssistanceConstants.StaticPositionLeft.getY()
                        - drivetrain.getState().Pose.getY())
                    / (Constants.DriverAssistanceConstants.StaticPositionLeft.getX()
                        - drivetrain.getState().Pose.getX()));

    Logger.getGlobal()
        .log(
            Level.INFO,
            "Target distance right: "
                + (Constants.DriverAssistanceConstants.StaticPositionRight.getY()
                        - drivetrain.getState().Pose.getY())
                    / (Constants.DriverAssistanceConstants.StaticPositionRight.getX()
                        - drivetrain.getState().Pose.getX()));

    if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {

      if (Math.abs(
              (Constants.DriverAssistanceConstants.StaticPositionLeft.getY()
                      - drivetrain.getState().Pose.getY())
                  / (Constants.DriverAssistanceConstants.StaticPositionLeft.getX()
                      - drivetrain.getState().Pose.getX()))
          < Math.abs(
              (Constants.DriverAssistanceConstants.StaticPositionRight.getY()
                      - drivetrain.getState().Pose.getY())
                  / (Constants.DriverAssistanceConstants.StaticPositionRight.getX()
                      - drivetrain.getState().Pose.getX()))) {
        pathCommand =
            drivetrain.driveToPose(Constants.DriverAssistanceConstants.StaticPositionLeft);
      } else {
        pathCommand =
            drivetrain.driveToPose(Constants.DriverAssistanceConstants.StaticPositionRight);
      }
    } else {
      if (Math.abs(
              (Invert(Constants.DriverAssistanceConstants.StaticPositionLeft).getY()
                      - drivetrain.getState().Pose.getY())
                  / (Invert(Constants.DriverAssistanceConstants.StaticPositionLeft).getX()
                      - drivetrain.getState().Pose.getX()))
          < Math.abs(
              (Invert(Constants.DriverAssistanceConstants.StaticPositionRight).getY()
                      - drivetrain.getState().Pose.getY())
                  / (Invert(Constants.DriverAssistanceConstants.StaticPositionRight).getX()
                      - drivetrain.getState().Pose.getX()))) {
        pathCommand =
            drivetrain.driveToPose(Invert(Constants.DriverAssistanceConstants.StaticPositionLeft));
      } else {
        pathCommand =
            drivetrain.driveToPose(Invert(Constants.DriverAssistanceConstants.StaticPositionRight));
      }
    }

    if (pathCommand != null) {
      CommandScheduler.getInstance().schedule(pathCommand);
    }
  }

  public void end(boolean interupt) {
    if (pathCommand != null) {
      pathCommand.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    if (pathCommand != null) {
      return pathCommand.isFinished();
    }
    return false;
  }
}
