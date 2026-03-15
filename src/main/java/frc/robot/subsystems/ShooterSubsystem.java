package frc.robot.subsystems;

import java.rmi.server.RMIClassLoader;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.ejml.equation.IntegerSequence.Range;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterSubsystemConstants;
import frc.robot.generated.TunerConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex shootMotorTop;
  private final SparkFlex shootFollowTop;
  private final SparkFlex shootMotorBottom;
  private final SparkFlex shootFollowBottom;

  private final SparkFlexConfig tFollowerConfig;
  private final SparkFlexConfig bFollowerConfig;
  private final SparkFlexConfig tShootConfig;
  private final SparkFlexConfig bShootConfig;

  // private final PIDController finalShootPID;

  private SparkClosedLoopController tShootClosedLoopController;
  private SparkClosedLoopController bSparkClosedLoopController;

  private double setVelocityTop;
  private double setVelocityBottom;
  private final double RPM_AT_8FT;
  private final double RANGE_AT_4100;
  private final double RPM_PER_FOOT;

  private final double robotX;
  private final double robotY;
  
  private double setVelocityKicker;

  private final SparkFlex kickMotor;
  private final SparkFlexConfig kickConfig;
  private final SparkClosedLoopController kickClosedLoopController;

  private final CommandSwerveDrivetrain ourDriveTrain;
  private final SwerveDriveKinematics m_Kinematics;

  public ShooterSubsystem(CommandSwerveDrivetrain ourDriveTrain) {

    SmartDashboard.putNumber("Velocity Top", 0);
    SmartDashboard.putNumber("Velocity Bottom", 0);
    SmartDashboard.putNumber("Current Limit", 0);

    this.ourDriveTrain = ourDriveTrain;
    m_Kinematics = new SwerveDriveKinematics(ourDriveTrain.getModuleLocations());

    RPM_AT_8FT = 4100;
    RANGE_AT_4100 = 8;
    RPM_PER_FOOT = 100;

    robotX = VisionSubsystem.getRobotPoseInFieldSpace().getX();
    robotY = VisionSubsystem.getRobotPoseInFieldSpace().getY();
    setVelocityTop = 4200;
    setVelocityBottom = 4200;
    setVelocityTop = 4100;
    setVelocityBottom = 4100;
    setVelocityKicker = 3000;

    shootMotorTop = new SparkFlex(13, MotorType.kBrushless);
    shootMotorBottom = new SparkFlex(15, MotorType.kBrushless);
    shootFollowTop = new SparkFlex(14, MotorType.kBrushless);
    shootFollowBottom = new SparkFlex(16, MotorType.kBrushless);
    kickMotor = new SparkFlex(31, MotorType.kBrushless);

    bFollowerConfig = new SparkFlexConfig();
    tFollowerConfig = new SparkFlexConfig();
    tShootConfig = new SparkFlexConfig();
    bShootConfig = new SparkFlexConfig();
    // commented to add when we know kicker runs
    kickConfig = new SparkFlexConfig();

    tShootClosedLoopController = shootMotorTop.getClosedLoopController();
    bSparkClosedLoopController = shootMotorBottom.getClosedLoopController();
    // commented to add when we know kicker runs
    kickClosedLoopController = kickMotor.getClosedLoopController();

    tShootConfig.smartCurrentLimit(40);
    bShootConfig.smartCurrentLimit(40);
    // commented to add when we know kicker runs
    kickConfig.smartCurrentLimit(40);

    // This might cause issues
    // tShootConfig.encoder.positionConversionFactor(1);
    // tShootConfig.encoder.positionConversionFactor(1);

    tShootConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(ShooterSubsystemConstants.shootPTop)
        .i(ShooterSubsystemConstants.shootITop)
        .d(0.0000)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(ShooterSubsystemConstants.shootPTop, ClosedLoopSlot.kSlot1)
        .i(ShooterSubsystemConstants.shootITop, ClosedLoopSlot.kSlot1)
        .d(0.0000, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        .feedForward
        // kV is now in Volts, so we multiply by the nominal voltage (12V)
        .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

    bShootConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(ShooterSubsystemConstants.shootPBot)
        .i(ShooterSubsystemConstants.shootIBot)
        .d(0.0000)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(ShooterSubsystemConstants.shootPBot, ClosedLoopSlot.kSlot1)
        .i(ShooterSubsystemConstants.shootIBot, ClosedLoopSlot.kSlot1)
        .d(0.0000, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        .feedForward
        // kV is now in Volts, so we multiply by the nominal voltage (12V)
        .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

    // commented to add when we know kicker runs
    // 0.00039
    // 0.0000016
    kickConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        // 0.00039
        .p(0.00034)
        // 0.000002
        .i(0.0000009)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        .feedForward
        // kV is now in Volts, so we multiply by the nominal voltage (12V)
        .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

    shootMotorTop.configure(
        tShootConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    shootMotorBottom.configure(
        bShootConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    // commented to add when we know kicker runs
    kickMotor.configure(
        kickConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  // Shoot balls. None adjustable velocity
  public void Shoot() {

    ChassisSpeeds speeds = m_Kinematics.toChassisSpeeds(ourDriveTrain.getModuleStates());
    // shootMotorBottom.set(-1);
    //shootMotorTop.set(1);
    //We need Feet per second, the x ft per second, the y fet per second, flight time, and min and max
    double target = rpmToHitTarget(robotX, robotY, MetersToFeet(6.4), MetersToFeet(6), speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, RPM_PER_FOOT, RPM_AT_8FT, RANGE_AT_4100);
    tShootClosedLoopController.setSetpoint(target, ControlType.kVelocity);
    bSparkClosedLoopController.setSetpoint(target, ControlType.kVelocity);
    
  }

  // Used to kick the balls up from the storage up into the shooter
  public void KickBalls() {
    // kickMotor.set(1);
    // commented to add when we know kicker runs
    kickClosedLoopController.setSetpoint(setVelocityKicker, ControlType.kVelocity);
  }

  public void StopShoot() {

    shootMotorTop.setVoltage(0);
    shootMotorBottom.setVoltage(0);
    kickMotor.setVoltage(0);
  }

  public double GetRPM() {
    return shootFollowTop.getEncoder().getVelocity();
  }

  public double GetWantedVelocity() {
    return setVelocityTop;
  }

  public double rpmToHitTarget(double robotXFT, double robotYFT, double targetXFT, double targetYFT, double vxMPerSec, double vyMPerSec, double flightTimeSec, double minRpm, double maxRpm){
    double dx = targetXFT - robotXFT;
    double dy = targetYFT - robotYFT;
    double distanceFt = Math.hypot(dx, dy);

    double vxFtPerSec = MetersToFeet(vxMPerSec);
    double vyFTPerSec = MetersToFeet(vyMPerSec);

    if(distanceFt < 1e-6){
      //This is when we are smack up against the hub
      return Math.max(minRpm, Math.min(maxRpm, RPM_AT_8FT));
    }
    double ux = dx / distanceFt;
    double uy = dy / distanceFt;

    double vParallel = vxFtPerSec * ux + vyFTPerSec * uy;
    double stationaryEquivalentRangeFt = distanceFt - vParallel * flightTimeSec;

    double rpmToHitTarget = RPM_AT_8FT + RPM_PER_FOOT * (stationaryEquivalentRangeFt - RANGE_AT_4100);

    return Math.max(minRpm, Math.min(maxRpm, rpmToHitTarget));
  }

  private double MetersToFeet(double meters){
    return meters * 3.2808399;
  }

  private double getXvPerFt(double vel, boolean GettingVy){
    if(GettingVy){
      return Math.cos(160) * vel;
    }

    return Math.sin(160) * vel;
  }
}
