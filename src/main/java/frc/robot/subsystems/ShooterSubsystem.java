package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex shootMotorLeft;
  private final SparkFlex leftFollow;
  private final SparkFlex shootMotorRight;
  private final SparkFlex rightFollow;

  private final SparkFlexConfig lFollowerConfig;
  private final SparkFlexConfig rFollowerConfig;
  private final SparkFlexConfig lShootConfig;
  private final SparkFlexConfig rShootConfig;

  private final PIDController finalShootPID;
  private SimpleMotorFeedforward motorFeedforward;
  private final AbsoluteEncoder shootEncoder;
  private final SparkFlex kickMotor;

  public ShooterSubsystem() {
    shootMotorLeft = new SparkFlex(0, MotorType.kBrushless);
    shootMotorRight = new SparkFlex(0, MotorType.kBrushless);
    leftFollow = new SparkFlex(0, MotorType.kBrushless);
    rightFollow = new SparkFlex(0, MotorType.kBrushless);

    lFollowerConfig = new SparkFlexConfig();
    rFollowerConfig = new SparkFlexConfig();
    lShootConfig = new SparkFlexConfig();
    rShootConfig = new SparkFlexConfig();

    motorFeedforward = new SimpleMotorFeedforward(1, 3, 2);
    finalShootPID = new PIDController(1, 1, 1);
    kickMotor = new SparkFlex(0, MotorType.kBrushless);
    SmartDashboard.putNumber("Velocity", 0);
    SmartDashboard.putNumber("Acceleration", 0);
    SmartDashboard.putNumber("Current Limit", 0);
    shootEncoder = shootMotorLeft.getAbsoluteEncoder();

    lFollowerConfig.inverted(true);
    lFollowerConfig.follow(shootMotorLeft);
    rFollowerConfig.inverted(true);
    rFollowerConfig.follow(shootMotorRight);

    lShootConfig.smartCurrentLimit((int) SmartDashboard.getNumber("Current Limit", 20));
    rShootConfig.smartCurrentLimit((int) SmartDashboard.getNumber("Current Limit", 20));

    leftFollow.configure(
        lFollowerConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    rightFollow.configure(
        rFollowerConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  // Shoot balls. None adjustable velocity
  public void Shoot() {
    shootMotorLeft.setVoltage(
        motorFeedforward.calculate(
                SmartDashboard.getNumber("Velocity", 0),
                SmartDashboard.getNumber("Acceleration", 0))
            + finalShootPID.calculate(
                shootEncoder.getVelocity(), SmartDashboard.getNumber("Velocity", 0)));
    shootMotorRight.setVoltage(
        motorFeedforward.calculate(
                SmartDashboard.getNumber("Velocity", 0),
                SmartDashboard.getNumber("Acceleration", 0))
            + finalShootPID.calculate(
                shootEncoder.getVelocity(), SmartDashboard.getNumber("Velocity", 0)));
  }

  // Used to kick the balls up from the storage up into the shooter
  public void KickBalls() {
    if (shootEncoder.getVelocity() == SmartDashboard.getNumber("Velocity", 0)) {
      kickMotor.set(1);
    } else {
      Shoot();
    }
  }

  public void StopShoot() {
    shootMotorLeft.set(0);
    shootMotorLeft.setVoltage(0);
    shootMotorRight.set(0);
    shootMotorRight.setVoltage(0);
  }
}
