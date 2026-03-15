package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterSubsystemConstants;

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
  private double setVelocityKicker;

  private final SparkFlex kickMotor;
  private final SparkFlexConfig kickConfig;
  private final SparkClosedLoopController kickClosedLoopController;

  public ShooterSubsystem() {

    SmartDashboard.putNumber("Velocity Top", 0);
    SmartDashboard.putNumber("Velocity Bottom", 0);
    SmartDashboard.putNumber("Current Limit", 0);
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

    tShootClosedLoopController.setSetpoint(setVelocityTop, ControlType.kVelocity);
    bSparkClosedLoopController.setSetpoint(setVelocityBottom, ControlType.kVelocity);
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
}
