package frc.robot.subsystems;

import org.opencv.features2d.FlannBasedMatcher;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterSubsystemConstants;
import frc.robot.Constants.VisionSubsystemConstants;

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

  private double wantedVelocity;
  private final double RPM_AT_8FT;
  private final double RPM_PER_FOOT;
  private final double RPM_PER_SLOW_FOOT;
  private final double rpmManual;

  private final CommandSwerveDrivetrain ourDriveTrain;
  private final SwerveDriveKinematics m_Kinematics;
  public boolean isShooting = false;
  public static boolean constantShootingOn = true; 

  public ShooterSubsystem(CommandSwerveDrivetrain ourDriveTrain) {

    SmartDashboard.putNumber("Velocity Top", 0);
    SmartDashboard.putNumber("Velocity Bottom", 0);
    SmartDashboard.putNumber("Current Limit", 0);

    this.ourDriveTrain = ourDriveTrain;
    m_Kinematics = new SwerveDriveKinematics(ourDriveTrain.getModuleLocations());

    RPM_AT_8FT = 4100;
    RPM_PER_FOOT = 400;
    RPM_PER_SLOW_FOOT = 300;
    rpmManual = 3500;
    wantedVelocity = 4100;

    shootMotorTop = new SparkFlex(13, MotorType.kBrushless);
    shootMotorBottom = new SparkFlex(15, MotorType.kBrushless);
    shootFollowTop = new SparkFlex(14, MotorType.kBrushless);
    shootFollowBottom = new SparkFlex(16, MotorType.kBrushless);

    bFollowerConfig = new SparkFlexConfig();
    tFollowerConfig = new SparkFlexConfig();
    tShootConfig = new SparkFlexConfig();
    bShootConfig = new SparkFlexConfig();
    // commented to add when we know kicker runs

    tShootClosedLoopController = shootMotorTop.getClosedLoopController();
    bSparkClosedLoopController = shootMotorBottom.getClosedLoopController();
    // commented to add when we know kicker runs

    tShootConfig.smartCurrentLimit(40);
    bShootConfig.smartCurrentLimit(40);
    // commented to add when we know kicker runs

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

    shootMotorTop.configure(
        tShootConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    shootMotorBottom.configure(
        bShootConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    // commented to add when we know kicker runs

  }

  @Override
  public void periodic() {
    if (shootMotorTop != null) {
      SmartDashboard.putNumber("ShooterRPM", shootMotorTop.getEncoder().getVelocity());
    }

    SmartDashboard.putBoolean("Maintaining Standby", !isShooting);
    SmartDashboard.putBoolean("CONSTANT SHOOTING", constantShootingOn);
  }

  public void setSpeed(double setpoint) {
    tShootClosedLoopController.setSetpoint(setpoint, ControlType.kVelocity);
    bSparkClosedLoopController.setSetpoint(setpoint, ControlType.kVelocity);
    SmartDashboard.putNumber("Shooter Velocity Setpoint", setpoint);
  }

  // Shoot balls. None adjustable velocity
  public void Shoot(boolean manual) {
    if (manual) {
      tShootClosedLoopController.setSetpoint(rpmManual, ControlType.kVelocity);
      bSparkClosedLoopController.setSetpoint(rpmManual, ControlType.kVelocity);
      SmartDashboard.putNumber("Shooter Velocity Setpoint", rpmManual);
    } else {
      double robotX = ourDriveTrain.getState().Pose.getX();
      double robotY = ourDriveTrain.getState().Pose.getY();
      double[] robotPose = new double[2];
      double[] targetPose = new double[2];
      // double[] targetPose = {MetersToFeet(VisionSubsystemConstants.RedHUBCenter[0]),
      // MetersToFeet(VisionSubsystemConstants.RedHUBCenter[1])};

      if (DriverStation.getAlliance().isPresent()) {

        // if so then branch for those 2 alliances
        // does atan of HUB.y - Robot.y / HUB.x - Robot.x and returns the resulting angle in degrees

        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
          targetPose[0] = 15.092;
          targetPose[1] = 13.255;

          robotPose[0] = MetersToFeet(robotX);
          robotPose[1] = MetersToFeet(robotY);
        } else if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
          targetPose[0] = MetersToFeet(VisionSubsystemConstants.RedHUBCenter[0]);
          targetPose[1] = MetersToFeet(VisionSubsystemConstants.RedHUBCenter[1]);

          robotPose[0] = MetersToFeet(robotX);
          robotPose[1] = MetersToFeet(robotY);
        }

        //double[] robotSpeed = {MetersToFeet(ourDriveTrain.getState().Speeds.vxMetersPerSecond), MetersToFeet(ourDriveTrain.getState().Speeds.vyMetersPerSecond)};
        // Logger.getGlobal()
        //    .log(Level.INFO, "robot x : " + robotPose[0] + " robot y : " + robotPose[1]);
        // Logger.getGlobal()
        //    .log(Level.INFO, "target x: " + targetPose[0] + "target y : " + targetPose[1]);
        double target = rpmToHitTarget(robotPose, targetPose);
        SmartDashboard.putNumber("Shooter Velocity Setpoint", target);
        tShootClosedLoopController.setSetpoint(target, ControlType.kVelocity);
        bSparkClosedLoopController.setSetpoint(target, ControlType.kVelocity);

        //   switch (DriverStation.getAlliance().get()) {
        //     case Blue:
        //       targetPose[0] = 15.092;
        //       targetPose[1] = 13.255;

        //       robotPose[0] = MetersToFeet(robotX);
        //       robotPose[1] = MetersToFeet(robotY);

        //     case Red:
        //       targetPose[0] = MetersToFeet(VisionSubsystemConstants.RedHUBCenter[0]);
        //       targetPose[1] = MetersToFeet(VisionSubsystemConstants.RedHUBCenter[1]);

        //       robotPose[0] = MetersToFeet(robotX);
        //       robotPose[1] = MetersToFeet(robotY);
        //   }
        // }

      }
    }
  }

  public void StopShoot() {

    SmartDashboard.putNumber("Shooter Velocity Setpoint", 0);
    shootMotorTop.setVoltage(0);
    shootMotorBottom.setVoltage(0);
  }

  public double GetRPM() {
    return shootMotorBottom.getEncoder().getVelocity();
  }

  public double GetWantedVelocity(boolean isManual) {
    if (isManual) {

      return rpmManual;
    }
    return wantedVelocity;
  }

  public double rpmToHitTarget(double[] robotPoseFt, double[] targetPoseFt, double[] robotSpeedFt) {

    double dx = robotPoseFt[0] - targetPoseFt[0];
    double dy = robotPoseFt[1] - targetPoseFt[1];
    double distanceFt = Math.hypot(dx, dy);

    SmartDashboard.putNumber("Shooter Distance To HUB", distanceFt);

    //unit vecctor of the x and y
    double ux = dx/distanceFt;
    double uy = dy/distanceFt;

    double vParallel = robotSpeedFt[0] * ux + robotSpeedFt[1] * uy;

    double stationaryEquivalentRangeFt = distanceFt - vParallel * 1.8;

    double rmpToHit;

    if(distanceFt <= 8){
      rmpToHit = RPM_AT_8FT + ((stationaryEquivalentRangeFt - 8) * RPM_PER_SLOW_FOOT);
    }else{
      rmpToHit = RPM_AT_8FT + ((stationaryEquivalentRangeFt - 8) * RPM_PER_FOOT);
    }

    if (wantedVelocity != rmpToHit) {
      wantedVelocity = rmpToHit;
    }

    return rmpToHit;
  }

  
  public double rpmToHitTarget(double[] robotPoseFt, double[] targetPoseFt) {

    double dx = robotPoseFt[0] - targetPoseFt[0];
    double dy = robotPoseFt[1] - targetPoseFt[1];
    double distanceFt = Math.hypot(dx, dy);

    SmartDashboard.putNumber("Shooter Distance To HUB", distanceFt);


    double rmpToHit;

    if(distanceFt <= 8){
      rmpToHit = RPM_AT_8FT + ((distanceFt - 8) * RPM_PER_SLOW_FOOT);
    }else{
      rmpToHit = RPM_AT_8FT + ((distanceFt - 8) * RPM_PER_FOOT);
    }

    if (wantedVelocity != rmpToHit) {
      wantedVelocity = rmpToHit;
    }

    return rmpToHit;
  }


  public double MetersToFeet(double meters) {
    return meters * 3.2808399;
  }

}
