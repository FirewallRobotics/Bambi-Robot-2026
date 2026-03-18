package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
  private final double rpmManual;

  
  

  private final CommandSwerveDrivetrain ourDriveTrain;
  private final SwerveDriveKinematics m_Kinematics;

  public ShooterSubsystem(CommandSwerveDrivetrain ourDriveTrain) {

    SmartDashboard.putNumber("Velocity Top", 0);
    SmartDashboard.putNumber("Velocity Bottom", 0);
    SmartDashboard.putNumber("Current Limit", 0);

    this.ourDriveTrain = ourDriveTrain;
    m_Kinematics = new SwerveDriveKinematics(ourDriveTrain.getModuleLocations());

    RPM_AT_8FT = 4100;
    RPM_PER_FOOT = 300;
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

  // Shoot balls. None adjustable velocity
  public void Shoot(boolean manual) {
    if (manual) {
      tShootClosedLoopController.setSetpoint(rpmManual, ControlType.kVelocity);
      bSparkClosedLoopController.setSetpoint(rpmManual, ControlType.kVelocity);
    } else{
      double robotX = ourDriveTrain.getState().Pose.getX();
      double robotY = ourDriveTrain.getState().Pose.getY();
      double[] robotPose = {MetersToFeet(robotX), MetersToFeet(robotY)};
      double[] targetPose = {MetersToFeet(VisionSubsystemConstants.RedHUBCenter[0]), MetersToFeet(VisionSubsystemConstants.RedHUBCenter[1])};

      //Logger.getGlobal().log(Level.INFO, "Distance: " + rpmToHitTarget(robotPose, targetPose));

      //ChassisSpeeds speeds = m_Kinematics.toChassisSpeeds(ourDriveTrain.getModuleStates());
      // shootMotorBottom.set(-1);
      //shootMotorTop.set(1);
      //We need Feet per second, the x ft per second, the y fet per second, flight time, and min and max
      double target = rpmToHitTarget(robotPose, targetPose);
      tShootClosedLoopController.setSetpoint(target, ControlType.kVelocity);
      bSparkClosedLoopController.setSetpoint(target, ControlType.kVelocity);
    }
    
    
  }


  public void StopShoot() {

    shootMotorTop.setVoltage(0);
    shootMotorBottom.setVoltage(0);
    
  }

  public double GetRPM() {
    return shootFollowTop.getEncoder().getVelocity();
  }

  public double GetWantedVelocity(boolean isManual) {
    if (isManual) {
      return rpmManual;
    }
    return wantedVelocity;
  }

  private double rpmToHitTarget(double[] robotPoseFt, double[] targetPoseFt) {

    double dx = robotPoseFt[0] - targetPoseFt[0];
    double dy = robotPoseFt[1] - targetPoseFt[1];
    double distanceFt = Math.hypot(dx, dy);

    double rmpToHit = RPM_AT_8FT + ((distanceFt - 8) * RPM_PER_FOOT);

    // Logger.getGlobal().log(Level.INFO, "distance: " + distanceFt);

    if (wantedVelocity != rmpToHit) {
      wantedVelocity = rmpToHit;
    }

    return rmpToHit;
  }

  // public double rpmToHitTarget(double robotXM, double robotYM, double targetXM, double targetYM,
  // double vxMPerSec, double vyMPerSec, double flightTimeSec, double minRpm, double maxRpm){

  //   double robotXFT = MetersToFeet(robotXM);
  //   double robotYFT = MetersToFeet(robotYM);

  //   double targetXFT = MetersToFeet(targetXM);
  //   double targetYFT = MetersToFeet(targetYM);

  //   double dx = targetXFT - robotXFT;
  //   double dy = targetYFT - robotYFT;
  //   //Logger.getGlobal().log(Level.INFO, "target feet x: " + targetXFT);
  //   //Logger.getGlobal().log(Level.INFO, "target feet y: " + targetYFT);
  //   //Logger.getGlobal().log(Level.INFO, "robot feet x: " + robotXFT);
  //   //Logger.getGlobal().log(Level.INFO, "robot feet y: " + robotYFT);
  //   double distanceFt = Math.hypot(dx, dy);

  //   double vxFtPerSec = MetersToFeet(vxMPerSec);
  //   double vyFTPerSec = MetersToFeet(vyMPerSec);

  //   if(distanceFt < 1e-6){
  //     //This is when we are smack up against the hub
  //     return Math.max(minRpm, Math.min(maxRpm, RPM_AT_8FT));
  //   }
  //   double ux = dx / distanceFt;
  //   double uy = dy / distanceFt;

  //   //double vParallel = vxFtPerSec * ux + vyFTPerSec * uy;
  //   double stationaryEquivalentRangeFt = distanceFt;

  //   double rpmToHitTarget = RPM_AT_8FT + RPM_PER_FOOT * (stationaryEquivalentRangeFt -
  // RANGE_AT_4100);

  //   //Logger.getGlobal().log(Level.INFO, "feet: " + distanceFt);
  //   //Logger.getGlobal().log(Level.INFO, "velocity: " + Math.max(minRpm, Math.min(maxRpm,
  // rpmToHitTarget)));

  //   return Math.max(minRpm, Math.min(maxRpm, rpmToHitTarget));
  // }

  private double MetersToFeet(double meters) {
    return meters * 3.2808399;
  }

  // private double getXvPerFt(double vel, boolean GettingVy){
  //   if(GettingVy){
  //     return Math.cos(160) * vel;
  //   }

  //   return Math.sin(160) * vel;
  // }

  // private double squared(double a){
  //   return a * a;
  // }
}
