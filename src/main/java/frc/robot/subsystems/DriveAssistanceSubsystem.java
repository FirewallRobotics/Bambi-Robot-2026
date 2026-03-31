package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.VisionSubsystemConstants;
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

    // if robot is enabled and the shoot command is not running
    if (RobotState.isEnabled() && !shooterSubsystem.isShooting) {

      // create the double lists for the robot pose and target pose
      double[] robotPose = new double[2];
      double[] targetPose = new double[2];

      // assign the robot pose
      robotPose[0] = shooterSubsystem.MetersToFeet(drivetrain.getState().Pose.getX());
      robotPose[1] = shooterSubsystem.MetersToFeet(drivetrain.getState().Pose.getY());

      // if we are on blue
      if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {

        // set the HUB position for blue
        targetPose[0] = 15.092;
        targetPose[1] = 13.255;

        // if the pose of our robot is within the zone
        if (drivetrain.getState().Pose.getX() < 4.5) {

          // get the RPM
          double target = shooterSubsystem.rpmToHitTarget(robotPose, targetPose);

          // set the RPM
          shooterSubsystem.setSpeed(target);

          // if we are not in the zone then disable the shooter
        } else {
          shooterSubsystem.setSpeed(0);
        }

        // if we are on red
      } else {

        // set the HUB position for red
        targetPose[0] = shooterSubsystem.MetersToFeet(VisionSubsystemConstants.RedHUBCenter[0]);
        targetPose[1] = shooterSubsystem.MetersToFeet(VisionSubsystemConstants.RedHUBCenter[1]);

        // if the pose of our robot is within the zone
        if (drivetrain.getState().Pose.getX() > 12) {

          // get the RPM
          double target = shooterSubsystem.rpmToHitTarget(robotPose, targetPose);

          // set the RPM
          shooterSubsystem.setSpeed(target);

          // if we are not in the zone then disable the shooter
        } else {
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
