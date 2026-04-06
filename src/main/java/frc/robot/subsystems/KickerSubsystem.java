package frc.robot.subsystems;

import com.andymark.jni.AM_CAN_Color_Sensor;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {
  private final SparkFlex kickerMotor;

  private double setVelocityKicker;
  private double panicVelocity;

  AM_CAN_Color_Sensor ballSensor;

  private final SparkFlexConfig kickConfig;
  private final SparkClosedLoopController kickClosedLoopController;

  public static boolean ColorKickerLockout = false;

  public KickerSubsystem() {
    kickerMotor = new SparkFlex(31, MotorType.kBrushless);
    setVelocityKicker = 4500;
    panicVelocity = -3000;

    ballSensor = new AM_CAN_Color_Sensor(43);

    ballSensor.turnLedOn();

    SmartDashboard.putBoolean("PreloadBalls", true);

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

  @Override
  public void periodic() {
    if (kickerMotor != null) {
      // SmartDashboard.putNumber("KickerRPM", kickerMotor.getEncoder().getVelocity());
    }

    // if we are not locked out start the main logic
    // any kicker command locks us out
    if (!ColorKickerLockout) {

      // if the sensor sees a ball closer then the prox and the driver has allowed preloading
      if (ballSensor.getData().proximity > Constants.KickerSubsystemConstants.MaxProx
          && SmartDashboard.getBoolean("PreloadBalls", false)) {

        // say the ball sensor doesn't see anything
        SmartDashboard.putBoolean("BallSensor", false);

        // kick balls at 1500 RPM
        KickBalls(1500);

      // or if the ball sensor is under the prox, the driver has allowed preloading, and the sensor is off
      } else if (ballSensor.getData().proximity < Constants.KickerSubsystemConstants.MaxProx
          && SmartDashboard.getBoolean("PreloadBalls", false)
          && !SmartDashboard.getBoolean("BallSensor", false)) {

        // say the sensor sees something
        SmartDashboard.putBoolean("BallSensor", true);

        // stop the kicker
        stopKicker();
      }
    }
  }

  // Used to kick the balls up from the storage up into the shooter
  public void KickBalls(double setpoint) {
    // kickMotor.set(1);
    // commented to add when we know kicker runs
    kickClosedLoopController.setSetpoint(setpoint, ControlType.kVelocity);
    // SmartDashboard.putNumber("KickerRPMSetpoint", setVelocityKicker);
  }

  // Used to kick the balls up from the storage up into the shooter
  public void KickBalls() {
    ColorKickerLockout = true;
    // kickMotor.set(1);
    // commented to add when we know kicker runs
    kickClosedLoopController.setSetpoint(setVelocityKicker, ControlType.kVelocity);
    // SmartDashboard.putNumber("KickerRPMSetpoint", setVelocityKicker);
  }

  public void panicKickBalls() {
    ColorKickerLockout = true;
    kickClosedLoopController.setSetpoint(panicVelocity, ControlType.kVelocity);
  }

  public void stopKicker() {
    kickerMotor.setVoltage(0);
    // SmartDashboard.putNumber("KickerRPMSetpoint", 0);
  }
}
