package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveAssistanceSubsystem extends SubsystemBase {

    private PhoenixPIDController turnpPidController;
    private CommandSwerveDrivetrain drivetrain;

    public DriveAssistanceSubsystem(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;

        turnpPidController = new PhoenixPIDController(0, 0, 0);
    }

    public void driveFacingHUB(double vx, double vy){
        SwerveRequest.RobotCentricFacingAngle request = new RobotCentricFacingAngle();
        request.HeadingController = turnpPidController;
        request.TargetDirection = new Rotation2d(Math.toRadians(VisionSubsystem.getAngleToHUB()));
        request.VelocityX = vx;
        request.VelocityY = vy;

        drivetrain.applyRequest(() -> request);
    }
}
