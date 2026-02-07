package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkFlex arm_motor;
  private final SparkFlex intake_motor;
  private ArmFeedforward armFeedfoward;

  SparkClosedLoopController controller;

  private ArmFeedforward feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts,
          ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad,
          ArmConstants.kAVoltSecondSquaredPerRad);

  public IntakeSubsystem() {
    intake_motor = new SparkFlex(ArmConstants.IntakeMotorID, MotorType.kBrushless);
    arm_motor = new SparkFlex(ArmConstants.ArmMotorID, MotorType.kBrushless);

    SparkFlexConfig motorConfig = new SparkFlexConfig();

    motorConfig.smartCurrentLimit(20);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.closedLoop.pid(0f, 0f, 0f, ClosedLoopSlot.kSlot0);

    arm_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    controller = arm_motor.getClosedLoopController();

    armFeedfoward =
        new ArmFeedforward(
            ArmConstants.kSVolts,
            ArmConstants.kGVolts,
            ArmConstants.kVVoltSecondPerRad,
            ArmConstants.kAVoltSecondSquaredPerRad);
  }

  @Override
  public void periodic(){}

  /** Hold intake arm at current position using feed forward (runs once) */
  public void holdUp(Double position, Double velocity) {
    arm_motor.set(armFeedfoward.calculate(position, velocity));
  }

  /** Manually set the speed of the intake arm */
  public void angleArm(Double speed) {
    arm_motor.set(speed);
  }

  /** Hold the arm at a specified target position */
  // TODO: NEEDS TESTING
  public void angleArmPID(double targetPosition){
    State setpoint = new State(targetPosition, 0);
    double ff = feedforward.calculate(setpoint.position * 2 * Math.PI, setpoint.velocity);
    controller.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
  }

  public void StartIntake(Double speed) {
    intake_motor.set(speed);
  }

  public void StopIntake() {
    intake_motor.set(0);
  }
}
