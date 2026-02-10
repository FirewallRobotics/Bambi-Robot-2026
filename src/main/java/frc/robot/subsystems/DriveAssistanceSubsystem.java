package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import java.util.function.Supplier;

public class DriveAssistanceSubsystem extends SubsystemBase {

  private PhoenixPIDController turnpPidController;
  private CommandSwerveDrivetrain drivetrain;
  private RobotContainer robotContainer;

  private final SwerveRequest.FieldCentricFacingAngle alignRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(DriveConstants.maxSpeed * 0.1) // Add a 10% deadband to translation only
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public DriveAssistanceSubsystem(
      CommandSwerveDrivetrain drivetrain, RobotContainer robotContainer) {
    this.drivetrain = drivetrain;
    this.robotContainer = robotContainer;

    turnpPidController = new PhoenixPIDController(0, 0, 0);
  }

  private Command prepShot(Supplier<Pose2d> targetPose) {
    return alignDrive(robotContainer.joystick, targetPose, robotContainer.drivetrain);
  }

  public Command prepFerryShot() {
    // return prepShot(() ->
    // DriveConstants.getFerryPose(drive.getState().Pose.getTranslation()).toPose2d());
    return alignDrive(
        robotContainer.joystick,
        () ->
            DriveConstants.getFerryPose(robotContainer.drivetrain.getState().Pose.getTranslation())
                .toPose2d(),
        robotContainer.drivetrain);
  }

  public Command prepHubShot() {
    return prepShot(() -> DriveConstants.getHubPose().toPose2d());
  }

  public Command alignDrive(
      CommandXboxController controller,
      Supplier<Pose2d> targetPoseSupplier,
      CommandSwerveDrivetrain drivetrain) {
    return drivetrain.applyRequest(
        () -> {
          double controllerVelX = -controller.getLeftY();
          double controllerVelY = -controller.getLeftX();

          Pose2d drivePose = drivetrain.getState().Pose;
          Pose2d targetPose = targetPoseSupplier.get();
          double shooterOffset = -DriveConstants.shooterSideOffset.in(Units.Meters);
          double targetDistance =
              drivePose.getTranslation().getDistance(targetPose.getTranslation());
          double shooterAngleRads = Math.acos(shooterOffset / targetDistance);
          Rotation2d shooterAngle = Rotation2d.fromRadians(shooterAngleRads);
          Rotation2d offsetAngle = Rotation2d.kCCW_90deg.minus(shooterAngle);
          Rotation2d shooterAngleOffset = Rotation2d.fromDegrees(2);
          Rotation2d desiredAngle =
              offsetAngle
                  .plus(drivePose.relativeTo(targetPose).getTranslation().getAngle())
                  .plus(shooterAngleOffset);
          Rotation2d currentAngle = drivePose.getRotation();
          Rotation2d deltaAngle = currentAngle.minus(desiredAngle);
          double wrappedAngleDeg = MathUtil.inputModulus(deltaAngle.getDegrees(), -180.0, 180.0);
          SmartDashboard.putNumber("ANGLE", desiredAngle.getDegrees());

          if ((Math.abs(wrappedAngleDeg)
                  < DriveConstants.epsilonAngleToGoal.in(Units.Degrees)) // if facing goal already
              && Math.hypot(controllerVelX, controllerVelY) < 0.1) {
            return new SwerveRequest.SwerveDriveBrake();
          } else {
            alignRequest.TargetDirection = desiredAngle;
            alignRequest.HeadingController = new PhoenixPIDController(2.25, 0, 0);
            alignRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
            return alignRequest
                .withVelocityX(
                    controllerVelX
                        * DriveConstants.maxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(
                    -controller.getLeftX()
                        * DriveConstants.maxSpeed);
          }
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
