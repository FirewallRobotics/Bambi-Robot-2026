package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkFlex arm_motor;
  private final SparkFlex intake_motor;
  private ArmFeedforward armFeedfoward;

  private boolean PIDcontrol;
  private PIDController armPidController;
  private double PIDTargetPosition;

  public IntakeSubsystem() {
    intake_motor = new SparkFlex(ArmConstants.IntakeMotorID, MotorType.kBrushless);
    arm_motor = new SparkFlex(ArmConstants.ArmMotorID, MotorType.kBrushless);
  
    PIDcontrol = false;
    PIDTargetPosition = 0;

    armPidController = new PIDController(0, 0, 0);

    armFeedfoward =
        new ArmFeedforward(
            ArmConstants.kSVolts,
            ArmConstants.kGVolts,
            ArmConstants.kVVoltSecondPerRad,
            ArmConstants.kAVoltSecondSquaredPerRad);
  }

  @Override
  public void periodic(){

    // checks if the arm is under PID control
    // if so then evaluate and apply the PID
    if(PIDcontrol){
      arm_motor.set(armPidController.calculate(arm_motor.getAbsoluteEncoder().getPosition(), PIDTargetPosition));
    }
  }

  /** Hold intake arm at current position using feed forward (runs once) */
  public void holdUp(Double position, Double velocity) {
    PIDcontrol = false;
    arm_motor.set(armFeedfoward.calculate(position, velocity));
  }

  /** Manually set the speed of the intake arm */
  public void angleArm(Double speed) {
    PIDcontrol = false;
    arm_motor.set(speed);
  }

  /** <b>Move the intake arm to a target position</b>
   * <p>will set the target position to be used as the setpoint in the subsystem periodic </p>
   */
  public void angleArmPID(double targetPosition){
    PIDcontrol = true;
    PIDTargetPosition = targetPosition;
  }

  public void StartIntake(Double speed) {
    intake_motor.set(speed);
  }

  public void StopIntake() {
    intake_motor.set(0);
  }
}
