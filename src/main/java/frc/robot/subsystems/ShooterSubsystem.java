package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex shootMotorTop;
  // private final SparkFlex leftFollow;
  private final SparkFlex shootMotorBottom;
  // private final SparkFlex rightFollow;

  // private final SparkFlexConfig lFollowerConfig;
  // private final SparkFlexConfig rFollowerConfig;
  private final SparkFlexConfig tShootConfig;
  private final SparkFlexConfig bShootConfig;

  // private final PIDController finalShootPID;
  private SimpleMotorFeedforward motorFeedforward;
  private final AbsoluteEncoder tShootEncoder;
  private final AbsoluteEncoder bShootEncoder;

  // private final SparkFlex kickMotor;

  public ShooterSubsystem() {
    shootMotorTop = new SparkFlex(13, MotorType.kBrushless);
    shootMotorBottom = new SparkFlex(15, MotorType.kBrushless);
    // leftFollow = new SparkFlex(0, MotorType.kBrushless);
    // rightFollow = new SparkFlex(0, MotorType.kBrushless);

    // lFollowerConfig = new SparkFlexConfig();
    // rFollowerConfig = new SparkFlexConfig();
    tShootConfig = new SparkFlexConfig();
    bShootConfig = new SparkFlexConfig();

    motorFeedforward = new SimpleMotorFeedforward(1, 3, 2);
    // finalShootPID = new PIDController(1, 1, 1);
    // kickMotor = new SparkFlex(0, MotorType.kBrushless);
    SmartDashboard.putNumber("Velocity Top", 0);
    SmartDashboard.putNumber("Velocity Bottom", 0);
    SmartDashboard.putNumber("Acceleration", 0);
    SmartDashboard.putNumber("Current Limit", 0);
    tShootEncoder = shootMotorTop.getAbsoluteEncoder();
    bShootEncoder = shootMotorBottom.getAbsoluteEncoder();

    // lFollowerConfig.inverted(true);
    // lFollowerConfig.follow(shootMotorLeft);
    // rFollowerConfig.inverted(true);
    // rFollowerConfig.follow(shootMotorRight);

    tShootConfig.smartCurrentLimit((int) SmartDashboard.getNumber("Current Limit", 20));
    bShootConfig.smartCurrentLimit((int) SmartDashboard.getNumber("Current Limit", 20));

    // tShootConfig.inverted(true);

    shootMotorTop.configure(
        tShootConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    shootMotorBottom.configure(
        bShootConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    // leftFollow.configure(
    //     lFollowerConfig,
    //     com.revrobotics.ResetMode.kNoResetSafeParameters,
    //     com.revrobotics.PersistMode.kPersistParameters);
    // rightFollow.configure(
    //     rFollowerConfig,
    //     com.revrobotics.ResetMode.kNoResetSafeParameters,
    //     com.revrobotics.PersistMode.kPersistParameters);
  }

  // Shoot balls. None adjustable velocity
  public void Shoot() {

    // shootMotorBottom.set(-1);
    // shootMotorTop.set(1);

    shootMotorTop.setVoltage(
        motorFeedforward.calculateWithVelocities(
            tShootEncoder.getVelocity(), SmartDashboard.getNumber("Velocity Top", 0)));
    shootMotorBottom.setVoltage(
        motorFeedforward.calculateWithVelocities(
            bShootEncoder.getVelocity(), SmartDashboard.getNumber("Velocity Bottom", 0)));

    // shootMotorLeft.setVoltage(
    //     motorFeedforward.calculate(
    //             SmartDashboard.getNumber("Velocity", 0),
    //             SmartDashboard.getNumber("Acceleration", 0))
    //         + finalShootPID.calculate(
    //             shootEncoder.getVelocity(), SmartDashboard.getNumber("Velocity", 0)));
    // shootMotorRight.setVoltage(
    //     motorFeedforward.calculate(
    //             SmartDashboard.getNumber("Velocity", 0),
    //             SmartDashboard.getNumber("Acceleration", 0))
    //         + finalShootPID.calculate(
    //             shootEncoder.getVelocity(), SmartDashboard.getNumber("Velocity", 0)));
  }

  // Used to kick the balls up from the storage up into the shooter
  public void KickBalls() {
    // if (shootEncoder.getVelocity() == SmartDashboard.getNumber("Velocity", 0)) {
    //   kickMotor.set(1);
    // } else {
    //   Shoot();
    // }
  }

  public void StopShoot() {

    shootMotorTop.setVoltage(0);
    shootMotorBottom.setVoltage(0);
    // shootMotorTop.set(0);
    // shootMotorBottom.set(0);

    // shootMotorLeft.set(0);
    // shootMotorLeft.setVoltage(0);
    // shootMotorRight.set(0);
    // shootMotorRight.setVoltage(0);
  }
}
