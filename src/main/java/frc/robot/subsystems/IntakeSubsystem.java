package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkFlex arm_motor;
  private final SparkFlex intake_motor;
  private ArmFeedforward armFeedfoward;

  public IntakeSubsystem() {
    intake_motor = new SparkFlex(ArmConstants.IntakeMotorID, MotorType.kBrushless);
    arm_motor = new SparkFlex(ArmConstants.ArmMotorID, MotorType.kBrushless);

    armFeedfoward =
        new ArmFeedforward(
            ArmConstants.kSVolts,
            ArmConstants.kGVolts,
            ArmConstants.kVVoltSecondPerRad,
            ArmConstants.kAVoltSecondSquaredPerRad);
  }

  public void holdUp(Double position, Double velocity) {
    arm_motor.set(armFeedfoward.calculate(position, velocity));
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
