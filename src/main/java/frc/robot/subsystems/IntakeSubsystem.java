package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final Spark motor;

  public IntakeSubsystem() {
    motor = new Spark(0);
  }

  public void StartIntake(Double speed) {
    motor.set(speed);
  }

  public void StopIntake() {
    motor.set(0);
  }
}
