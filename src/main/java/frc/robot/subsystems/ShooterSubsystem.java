package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ShooterSubsystem {
  private final SparkFlex shooterTopPortMotor;
  // private final SparkFlex shooterTopStarboardMotor;
  private final SparkFlex shooterBottomPortMotor;
  private final SparkFlex kickerPortMotor;

  private final SparkClosedLoopController topPortClosedLoopController;
  private final SparkClosedLoopController bottomPortClosedLoopController;

  private final SparkFlexConfig tPortConfig;
  // private final SparkFlexConfig tStarboardConfig;
  private final SparkFlexConfig bPortConfig;
  private final SparkFlexConfig kickerPortConfig;

  public ShooterSubsystem() {
    shooterTopPortMotor = new SparkFlex(1, MotorType.kBrushless);
    // shooterTopStarboardMotor = new SparkFlex(6, MotorType.kBrushless);
    shooterBottomPortMotor = new SparkFlex(4, MotorType.kBrushless);
    kickerPortMotor = new SparkFlex(3, MotorType.kBrushless);

    tPortConfig = new SparkFlexConfig();
    // tStarboardConfig = new SparkFlexConfig();
    bPortConfig = new SparkFlexConfig();
    kickerPortConfig = new SparkFlexConfig();

    tPortConfig.smartCurrentLimit(40);
    bPortConfig.smartCurrentLimit(40);
    tPortConfig.encoder.positionConversionFactor(1);
    bPortConfig.encoder.positionConversionFactor(1);
    topPortClosedLoopController = shooterTopPortMotor.getClosedLoopController();
    bottomPortClosedLoopController = shooterBottomPortMotor.getClosedLoopController();

    tPortConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.0000035)
        .i(0.1)
        .d(0)
        // daniel's fault
        .outputRange(-1, 1);
    bPortConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.0000035)
        .i(0.1)
        .d(0)
        .outputRange(-1, 1);

    // tStarboardConfig.follow(1, true);
    shooterTopPortMotor.configure(
        tPortConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    // shooterTopStarboardMotor.configure(
    // tStarboardConfig,
    // com.revrobotics.ResetMode.kNoResetSafeParameters,
    // com.revrobotics.PersistMode.kPersistParameters);
    shooterBottomPortMotor.configure(
        bPortConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    kickerPortMotor.configure(
        kickerPortConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  public void Shoot() {
    // shooterTopPortMotor.set(-0.3);
    topPortClosedLoopController.setSetpoint(2000, ControlType.kVelocity);
    bottomPortClosedLoopController.setSetpoint(-2000, ControlType.kVelocity);
    // shooterBottomPortMotor.set(0.3);
  }

  public void Upward() {}

  public void StopShoot() {
    // shooterTopPortMotor.set(0);
    // shooterBottomPortMotor.set(0);
    shooterTopPortMotor.setVoltage(0);
    shooterBottomPortMotor.setVoltage(0);
  }
}
