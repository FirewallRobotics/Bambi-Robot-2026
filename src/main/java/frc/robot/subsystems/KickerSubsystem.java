package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {
  private final SparkFlex kickerMotor;

  private double setVelocityKicker;

  private final SparkFlexConfig kickConfig;
  private final SparkClosedLoopController kickClosedLoopController;

  public KickerSubsystem() {
    kickerMotor = new SparkFlex(31, MotorType.kBrushless);
    setVelocityKicker = 4500;

    kickConfig = new SparkFlexConfig();
    kickClosedLoopController = kickerMotor.getClosedLoopController();
    kickConfig.smartCurrentLimit(40);

    kickConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        // 0.00039
        .p(0.00032)
        // 0.000009
        .i(0.00000085)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        .feedForward
        // kV is now in Volts, so we multiply by the nominal voltage (12V)
        .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

    kickerMotor.configure(
        kickConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  // Used to kick the balls up from the storage up into the shooter
  public void KickBalls() {
    // kickMotor.set(1);
    // commented to add when we know kicker runs
    kickClosedLoopController.setSetpoint(setVelocityKicker, ControlType.kVelocity);
  }

  public void stopKicker() {
    kickerMotor.setVoltage(0);
  }
}
