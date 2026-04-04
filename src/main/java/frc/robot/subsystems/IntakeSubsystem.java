package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {

  /** The primary intake motor */
  private final SparkFlex intakeMotor;

  /** The primary motor configuration */
  private final SparkFlexConfig intakeMotorConfig;

  /** The primary motor PIDF controller */
  private final SparkClosedLoopController intakeClosedLoop;

  // NOTE: The follower is defined in the physical motor and will not be defined here

  public IntakeSubsystem() {
    // get the controller with ID 41
    intakeMotor = new SparkFlex(41, MotorType.kBrushless);
    // create the config and PID objects to be changed
    intakeMotorConfig = new SparkFlexConfig();
    intakeClosedLoop = intakeMotor.getClosedLoopController();

    // set the current limit. Less is better but we need this to punch
    intakeMotorConfig.smartCurrentLimit(40);

    intakeMotorConfig
        .closedLoop
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

    // lastly configure the primary motor
    intakeMotor.configure(
        intakeMotorConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    if (intakeMotor != null) {
      // SmartDashboard.putNumber("IntakeRPM", intakeMotor.getEncoder().getVelocity());
    }
  }

  /** Makes the intake run at {@link IntakeSubsystemConstants#intakeVelocity} RPM */
  public void StartIntake() {
    intakeClosedLoop.setSetpoint(IntakeSubsystemConstants.intakeVelocity, ControlType.kVelocity);
  }

  /** Makes the intake stop moving by setting the output voltage to 0 */
  public void StopIntake() {
    intakeMotor.setVoltage(0);
  }
}
