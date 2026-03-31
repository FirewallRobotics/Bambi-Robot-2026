package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveAssistanceSubsystem extends SubsystemBase {

  private CommandSwerveDrivetrain drivetrain;
  private ShooterSubsystem shooterSubsystem;

  public DriveAssistanceSubsystem(
      CommandSwerveDrivetrain drivetrain,
      RobotContainer robotContainer,
      ShooterSubsystem shooterSubsystem) {
    this.drivetrain = drivetrain;
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void periodic() {
    if (RobotState.isEnabled() && !shooterSubsystem.isShooting) {
      if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
        if (drivetrain.getState().Pose.getX() < 4.5) {
          double dx =
              (drivetrain.getState().Pose.getX() * 3.2808399)
                  - (Constants.VisionSubsystemConstants.BlueHUBCenter[0] * 3.2808399);
          double dy =
              (drivetrain.getState().Pose.getY() * 3.2808399)
                  - (Constants.VisionSubsystemConstants.BlueHUBCenter[1] * 3.2808399);
          double distanceFt = Math.hypot(dx, dy);
          if (distanceFt <= 8) {
            shooterSubsystem.setSpeed(distanceFt * 300);
          } else {
            shooterSubsystem.setSpeed(distanceFt * 400);
          }
        }else{
          shooterSubsystem.setSpeed(0);
        }
      } else {
        if (drivetrain.getState().Pose.getX() > 12) {
          double dx =
              (drivetrain.getState().Pose.getX() * 3.2808399)
                  - (Constants.VisionSubsystemConstants.RedHUBCenter[0] * 3.2808399);
          double dy =
              (drivetrain.getState().Pose.getY() * 3.2808399)
                  - (Constants.VisionSubsystemConstants.RedHUBCenter[1] * 3.2808399);
          double distanceFt = Math.hypot(dx, dy);
          if (distanceFt <= 8) {
            shooterSubsystem.setSpeed(distanceFt * 300);
          } else {
            shooterSubsystem.setSpeed(distanceFt * 400);
          }
        }else{
          shooterSubsystem.setSpeed(0);
        }
      }
    }
  }

  /**
   * Vibrates the input controller if we are facing the HUB; will also vibrate in the direction for
   * it to move
   */
  public Command vibrateIfFaceingHUBDiscriptive(CommandXboxController joystick) {
    return run(() -> {
          if (Math.toRadians(VisionSubsystem.getAngleToHUB(drivetrain))
              == drivetrain.getState().Pose.getRotation().getRadians()) {
            joystick.setRumble(RumbleType.kBothRumble, 0.75);
          } else if (Math.toRadians(VisionSubsystem.getAngleToHUB(drivetrain))
              > drivetrain.getState().Pose.getRotation().getRadians()) {
            joystick.setRumble(RumbleType.kLeftRumble, 0.25);
          } else if (Math.toRadians(VisionSubsystem.getAngleToHUB(drivetrain))
              < drivetrain.getState().Pose.getRotation().getRadians()) {
            joystick.setRumble(RumbleType.kRightRumble, 0.25);
          } else {
            joystick.setRumble(RumbleType.kBothRumble, 0);
          }
        })
        .handleInterrupt(
            () -> {
              joystick.setRumble(RumbleType.kBothRumble, 0);
            });
  }

  /** Vibrates the input controller if we are facing the HUB */
  public Command vibrateIfFaceingHUB(CommandXboxController joystick) {
    return run(() -> {
          if (Math.toRadians(VisionSubsystem.getAngleToHUB(drivetrain))
              == drivetrain.getState().Pose.getRotation().getRadians()) {
            joystick.setRumble(RumbleType.kBothRumble, 0.75);
          } else {
            joystick.setRumble(RumbleType.kBothRumble, 0);
          }
        })
        .handleInterrupt(
            () -> {
              joystick.setRumble(RumbleType.kBothRumble, 0);
            });
  }
}
