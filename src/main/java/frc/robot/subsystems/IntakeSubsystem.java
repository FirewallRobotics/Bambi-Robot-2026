package frc.robot.subsystems;

<<<<<<< HEAD
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.IntakeSubsystemConstants;

public class IntakeSubsystem {

    private final SparkFlex intakeMotor;
    private final SparkFlexConfig intakeMotorConfig;
    private final SparkClosedLoopController intakeClosedLoop;

    public IntakeSubsystem(){
        intakeMotor = new SparkFlex(10, MotorType.kBrushless);
        intakeMotorConfig = new SparkFlexConfig();
        intakeClosedLoop = intakeMotor.getClosedLoopController();

        intakeMotorConfig.smartCurrentLimit(40);

        intakeMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed loop
            // slot, as it will default to slot 0.
            .p(IntakeSubsystemConstants.intakeP)
            .i(IntakeSubsystemConstants.intakeI)
            .d(0)
            .outputRange(-1, 1)
            // Set PID values for velocity control in slot 1
            .p(0, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
            .feedForward
            // kV is now in Volts, so we multiply by the nominal voltage (12V)
            .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

        intakeMotor.configure(
            intakeMotorConfig,
            com.revrobotics.ResetMode.kNoResetSafeParameters,
            com.revrobotics.PersistMode.kPersistParameters);
    }

    public void StartIntake(){
        intakeClosedLoop.setSetpoint(IntakeSubsystemConstants.intakeVelocity, ControlType.kVelocity);
    }

    public void StopIntake(){
        intakeMotor.setVoltage(0);
    }



=======
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
  public void periodic() {

    // checks if the arm is under PID control
    // if so then evaluate and apply the PID
    if (PIDcontrol) {
      arm_motor.set(
          armPidController.calculate(
              arm_motor.getAbsoluteEncoder().getPosition(), PIDTargetPosition));
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

  /**
   * <b>Move the intake arm to a target position</b>
   *
   * <p>will set the target position to be used as the setpoint in the subsystem periodic
   */
  public void angleArmPID(double targetPosition) {
    PIDcontrol = true;
    PIDTargetPosition = targetPosition;
  }

  public void StartIntake(Double speed) {
    intake_motor.set(speed);
  }

  public void StopIntake() {
    intake_motor.set(0);
  }
>>>>>>> 38945d14b5a7513796bbb71667b5f5196efa6d27
}
