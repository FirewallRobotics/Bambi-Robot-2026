package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ShooterSubsystem {
  private final SparkFlex shooterTopPortMotor;

  private final SparkFlexConfig tPortConfig;

  public ShooterSubsystem() {
    shooterTopPortMotor = new SparkFlex(1, MotorType.kBrushless);

    tPortConfig = new SparkFlexConfig();
  }

  public void Shoot() {
    shooterTopPortMotor.set(-0.3);
  }

  public void Upward() {}

  public void StopShoot() {
    shooterTopPortMotor.set(0);
  }
}
