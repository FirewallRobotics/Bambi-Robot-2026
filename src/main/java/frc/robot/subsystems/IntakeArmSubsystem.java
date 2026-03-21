package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArmSubsystem extends SubsystemBase {
  /** The primary arm motor */
  private final SparkFlex armMotor;

  /** The follower arm motor */
  private final SparkFlex followerMotor;

  /** The config of the primary arm motor */
  private final SparkFlexConfig armConfig;

  /** The config for the follower arm motor */
  private final SparkFlexConfig FollowerarmConfig;

  /** The PIDF controller of the primary arm motor (the follower doesn't use PIDedo) */
  private final SparkClosedLoopController armClosedLoopController;

  /** The raised position of the arm */
  private double upPosition;

  /** The lowered position of the arm (with a little extra to keep it down) */
  private double downPosition;

  public IntakeArmSubsystem() {
    // define the primary arm motor, its config, and the PID controller for it pending setting it up
    armMotor = new SparkFlex(17, MotorType.kBrushless);
    armConfig = new SparkFlexConfig();
    armClosedLoopController = armMotor.getClosedLoopController();

    // define the raised and lowered position (we give the lowered abit of extra so it pushes into
    // the bumpers and gets a better angle)
    upPosition = 0.18;
    downPosition = -0.04;

    // define the follower motor and its config
    followerMotor = new SparkFlex(18, MotorType.kBrushless);
    FollowerarmConfig = new SparkFlexConfig();

    // set a current limit on the follower
    FollowerarmConfig.smartCurrentLimit(40);

    // have it follow the primary
    FollowerarmConfig.follow(17, true);

    // configure the follower
    followerMotor.configure(
        FollowerarmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // set the current limit on the primary
    armConfig.smartCurrentLimit(40);

    // we don't have a abs encoder :( but I did calcuate the settings for the encoder and stored
    // them like this
    // armConfig.absoluteEncoder.positionConversionFactor(0.0725);
    // armConfig.absoluteEncoder.zeroOffset(0.275);

    // setup the PIDF for the primary
    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        .feedForward
        //  .kG(0)
        // .kS((0.36-0.35)/2)
        .kCos(0.35 + ((0.45 - 0.35) / 2));
    // .kCosRatio(1);

    // configure the primary motor
    armMotor.configure(
        armConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  /** returns if the arm is within a certain tolerence of its setpoint */
  public boolean isAtSetpoint(boolean goingUp) {
    if (goingUp && armMotor.getEncoder().getPosition() >= upPosition) {
      // Logger.getGlobal().log(Level.INFO, "Is at up setpoint");
      return true;

      // because the bottom position is offset into the bumper we have an arbitrary position to
      // allow it to find a setpoint
    } else if (!goingUp && armMotor.getEncoder().getPosition() <= 0.05) {
      // Logger.getGlobal().log(Level.INFO, "Is at down setpoint");
      return true;
    }
    return false;
  }

  public boolean isAtSetpoint(double setpoint) {
    return Math.abs(armMotor.getEncoder().getPosition() - setpoint) < 0.02;
  }

  /** Manually angles the intake arm without PID */
  public void AngleArm(boolean goingDown) {
    if (goingDown) {
      armMotor.set(-0.1);
    } else {
      armMotor.set(0.1);
    }
    // armMotor.setVoltage(SmartDashboard.getNumber("Hold Voltage", 0));
  }

  /** Angles the Intake arm to a custom position */
  public void AngleArmToSetpoint(double setpoint) {
    armClosedLoopController.setSetpoint(setpoint, ControlType.kPosition);
  }

  /** stops the intake arm by settings its output voltage to 0 */
  public void StopAngle() {
    armMotor.setVoltage(0);
  }

  /** moves the intake arm between 2 positions: rasied and lowered with PIDF */
  public void SetAngle(boolean isGoingUp) {
    if (isGoingUp) {
      armClosedLoopController.setSetpoint(upPosition, ControlType.kPosition);
    } else {
      armClosedLoopController.setSetpoint(downPosition, ControlType.kPosition);
    }
  }
}
