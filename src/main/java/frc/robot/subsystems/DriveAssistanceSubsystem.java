package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

public class DriveAssistanceSubsystem extends SubsystemBase {

  private CommandSwerveDrivetrain drivetrain;

  public DriveAssistanceSubsystem(
      CommandSwerveDrivetrain drivetrain, RobotContainer robotContainer) {
    this.drivetrain = drivetrain;
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
