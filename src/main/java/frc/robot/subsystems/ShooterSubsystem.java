package frc.robot.subsystems;

import java.rmi.server.RMIClassLoader;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
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
  private SimpleMotorFeedforward topMotorFeedforward;
  private SimpleMotorFeedforward bottomMotorFeedforward;
  private final RelativeEncoder tShootEncoder;
  private final RelativeEncoder bShootEncoder;

  private final PIDController tShootPID;
  private SparkClosedLoopController tShootClosedLoopController;

  private double setVelocity;
  

  // private final SparkFlex kickMotor;

  public ShooterSubsystem() {

    SmartDashboard.putNumber("Velocity Top", 0);
    SmartDashboard.putNumber("Velocity Bottom", 0);
    SmartDashboard.putNumber("Current Limit", 0);
    setVelocity = 1200;

    shootMotorTop = new SparkFlex(14, MotorType.kBrushless);
    shootMotorBottom = new SparkFlex(15, MotorType.kBrushless);
    // leftFollow = new SparkFlex(0, MotorType.kBrushless);
    // rightFollow = new SparkFlex(0, MotorType.kBrushless);

    // lFollowerConfig = new SparkFlexConfig();
    // rFollowerConfig = new SparkFlexConfig();
    tShootConfig = new SparkFlexConfig();
    bShootConfig = new SparkFlexConfig();

    tShootClosedLoopController = shootMotorTop.getClosedLoopController();

    topMotorFeedforward = new SimpleMotorFeedforward(1, 3, 2);
    bottomMotorFeedforward = new SimpleMotorFeedforward(1, 3, 2);
    
    // finalShootPID = new PIDController(1, 1, 1);
    // kickMotor = new SparkFlex(0, MotorType.kBrushless);
    
    tShootPID = new PIDController(0.01, 0, 0);
    tShootPID.setTolerance(100);

    

    tShootEncoder = shootMotorTop.getEncoder();
    bShootEncoder = shootMotorBottom.getEncoder();

    // lFollowerConfig.inverted(true);
    // lFollowerConfig.follow(shootMotorLeft);
    // rFollowerConfig.inverted(true);
    // rFollowerConfig.follow(shootMotorRight);

    tShootConfig.smartCurrentLimit(20);
    bShootConfig.smartCurrentLimit(20);

    // tShootConfig.inverted(true);
    tShootConfig.encoder.positionConversionFactor(1);
    tShootConfig.encoder.positionConversionFactor(1);

    tShootConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.001)
        .i(0.0000000001)
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
    //shootMotorTop.set(1);
    tShootClosedLoopController.setSetpoint(setVelocity, ControlType.kVelocity);
    //shootMotorTop.setVoltage(topMotorFeedforward.calculateWithVelocities(tShootEncoder.getVelocity(), setVelocity) + tShootPID.calculate(tShootEncoder.getVelocity(), setVelocity));

    // shootMotorTop.setVoltage(
    //     topMotorFeedforward.calculateWithVelocities(
    //         tShootEncoder.getVelocity(), 600));
    shootMotorBottom.setVoltage(
        bottomMotorFeedforward.calculateWithVelocities(
            bShootEncoder.getVelocity(), 600));
    //Logger.getGlobal().log(Level.INFO, "Top V: " + tShootEncoder.getVelocity());
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

    //shootMotorTop.setVoltage(0);
    //tShootClosedLoopController.setSetpoint(0, ControlType.kVelocity);
    
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
