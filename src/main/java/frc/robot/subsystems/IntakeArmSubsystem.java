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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystemConstants;

public class IntakeArmSubsystem extends SubsystemBase {
  private final SparkFlex armMotor;
  private final SparkFlex followerMotor;
  private final SparkFlexConfig armConfig;
  private final SparkFlexConfig FollowerarmConfig;
  private final SparkClosedLoopController armClosedLoopController;

  private double upPosition;
  private double downPosition;

  public IntakeArmSubsystem() {
    armMotor = new SparkFlex(17, MotorType.kBrushless);
    armConfig = new SparkFlexConfig();
    armClosedLoopController = armMotor.getClosedLoopController();
    upPosition = 0.31;
    downPosition = 0;

    followerMotor = new SparkFlex(18, MotorType.kBrushless);
    FollowerarmConfig = new SparkFlexConfig();

    FollowerarmConfig.smartCurrentLimit(40);

    FollowerarmConfig.follow(17, true);

    followerMotor.configure(
        FollowerarmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armConfig.smartCurrentLimit(40);

    armConfig.absoluteEncoder.positionConversionFactor(0.0725);
    armConfig.absoluteEncoder.zeroOffset(0.275);

    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.85)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        .feedForward
        .kG(0)
        .kS(0.1225)
        .kCos(0.4875)
        .kCosRatio(1);

    armMotor.configure(
        armConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  public void AngleArm(boolean goingDown) {
    // if(goingDown){
    //   armMotor.set(-0.1);
    // } else{
    //  armMotor.set(0.1);
    // }
    armMotor.setVoltage(SmartDashboard.getNumber("Hold Voltage", 0));
  }

  public void StopAngle() {
    armMotor.setVoltage(0);
  }

  public void SetAngle(boolean isGoingUp) {
    if (isGoingUp) {
      armClosedLoopController.setSetpoint(upPosition, ControlType.kPosition);
    } else {
      armClosedLoopController.setSetpoint(downPosition, ControlType.kPosition);
    }
  }
}
