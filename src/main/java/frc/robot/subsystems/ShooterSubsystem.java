package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final Spark motor;

  public ShooterSubsystem() {
    motor = new Spark(0);
  }

  // Shooting
  public void Shoot(Double speed) {
    motor.set(speed);
  }

  public void StopShoot() {
    motor.set(0);
  }
}
