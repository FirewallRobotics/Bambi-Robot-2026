package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final Spark arm_motor;
  private final Spark intake_motor;
  private ArmFeedforward armFeedfoward;

  public IntakeSubsystem() {
    intake_motor = new Spark(1);
    arm_motor = new Spark(2);
    armFeedfoward =
        new ArmFeedforward(
            ArmConstants.kSVolts,
            ArmConstants.kGVolts,
            ArmConstants.kVVoltSecondPerRad,
            ArmConstants.kAVoltSecondSquaredPerRad);
  }

  public void holdUp(Double position) {
    armFeedfoward.calculate(position, 2, 3);
  }

  public void angleArm(Double speed) {
    arm_motor.set(speed);
  }

  public void StartIntake(Double speed) {
    intake_motor.set(speed);
  }

  public void StopIntake() {
    intake_motor.set(0);
  }
}
