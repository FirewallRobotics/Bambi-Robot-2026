package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveAssistanceSubsystem extends SubsystemBase {

  private PhoenixPIDController turnpPidController;
  private CommandSwerveDrivetrain drivetrain;
  private RobotContainer robotContainer;

  public DriveAssistanceSubsystem(
      CommandSwerveDrivetrain drivetrain, RobotContainer robotContainer) {
    this.drivetrain = drivetrain;
    this.robotContainer = robotContainer;

    turnpPidController = new PhoenixPIDController(0, 0, 0);
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

  public void driveFacingHUB(double vx, double vy) {
    SwerveRequest.FieldCentricFacingAngle request = new FieldCentricFacingAngle();
    request.HeadingController = turnpPidController;
    request.TargetDirection =
        new Rotation2d(Math.toRadians(VisionSubsystem.getAngleToHUB(drivetrain)));
    request.VelocityX = vx;
    request.VelocityY = vy;

    drivetrain.applyRequest(() -> request);
  }

  public void maintainArchAroundHUB(double vx, double vy, CommandXboxController drivercontroller) {
    /**
     * Calculate distance to HUB using similar triangles: distance = (Y difference) / (X difference)
     * This assumes the HUB is at a fixed position relative to the robot's coordinate system
     */
    double M_mindist = Constants.DriverAssistanceConstants.minDistFeet * 0.3048;
    double M_maxdist = Constants.DriverAssistanceConstants.maxDistFeet * 0.3048;

    /**
     * Calculate current position and distance to HUB The distance calculation uses the ratio of Y/X
     * differences based on the alliance color (Blue/Red)
     */
    Pose2d currentpose = robotContainer.drivetrain.getState().Pose;
    double currentdistance = 0;

    double HUBAngle = VisionSubsystem.getAngleToHUB(drivetrain);

    /**
     * Calculate distance to HUB using similar triangles: For Blue alliance: (Y difference) / (X
     * difference) For Red alliance: (Y difference) / (X difference)
     */
    switch (DriverStation.getAlliance().get()) {
      case Blue:
        currentdistance =
            (Constants.VisionSubsystemConstants.BlueHUBCenter[1] - currentpose.getY())
                / (Constants.VisionSubsystemConstants.BlueHUBCenter[0] - currentpose.getX());
        break;
      case Red:
        currentdistance =
            (Constants.VisionSubsystemConstants.RedHUBCenter[1] - currentpose.getY())
                / (Constants.VisionSubsystemConstants.RedHUBCenter[0] - currentpose.getX());
        break;
    }

    /**
     * Redirect movement when approaching or moving away from HUB: 1. Calculate direction vector
     * using HUBAngle (in radians) 2. Scale velocity by 0.5 (tunable parameter) 3. Use trigonometry
     * to decompose velocity into X/Y components
     */
    if (currentdistance > M_maxdist) {
      // Redirect away from HUB
      double angleRad = Math.toRadians(HUBAngle);
      double vxRedirect = vx * Math.cos(angleRad);
      double vyRedirect = vy * Math.sin(angleRad);
      // drivercontroller.setRumble(RumbleType.kBothRumble, 0.5); Add later
      driveFacingHUB(vxRedirect, vyRedirect);
    } else if (currentdistance < M_mindist) {
      // Redirect towards HUB
      double angleRad = Math.toRadians(HUBAngle);
      // I don't know if this will work. Qwen wrote this
      double vxRedirect = -vx * Math.cos(angleRad);
      double vyRedirect = -vy * Math.sin(angleRad);
      // drivercontroller.setRumble(RumbleType.kBothRumble, 0.5);
      driveFacingHUB(vxRedirect, vyRedirect);
    } else {
      // drivercontroller.setRumble(RumbleType.kBothRumble, 0);
      driveFacingHUB(vx, vy);
    }
  }
}
