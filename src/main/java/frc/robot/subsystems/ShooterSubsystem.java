package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ShooterSubsystem {
  private final SparkFlex shooterTopPortMotor;
  private final SparkFlex shooterTopStarboardMotor;
  private final SparkFlex shooterBottomPortMotor;

  private final SparkFlexConfig tPortConfig;
  private final SparkFlexConfig tStarboardConfig;
  private final SparkFlexConfig bPortConfig;

  public ShooterSubsystem() {
    shooterTopPortMotor = new SparkFlex(1, MotorType.kBrushless);
    shooterTopStarboardMotor = new SparkFlex(6, MotorType.kBrushless);
    shooterBottomPortMotor = new SparkFlex(4, MotorType.kBrushless);

    tPortConfig = new SparkFlexConfig();
    tStarboardConfig = new SparkFlexConfig();
    bPortConfig = new SparkFlexConfig();

    tStarboardConfig.follow(1, true);
    shooterTopPortMotor.configure(
        tPortConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    shooterTopStarboardMotor.configure(
        tStarboardConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    shooterBottomPortMotor.configure(
        bPortConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  public void Shoot() {
    shooterTopPortMotor.set(-0.3);
    shooterBottomPortMotor.set(0.3);
  }

  public void Upward() {}

  public void StopShoot() {
    shooterTopPortMotor.set(0);
    shooterBottomPortMotor.set(0);
  }
}
