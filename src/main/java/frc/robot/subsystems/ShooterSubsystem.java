package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex finalShootMotor;
  private final PIDController finalShootPID;
  private SimpleMotorFeedforward motorFeedforward;
  private final AbsoluteEncoder shootEncoder;
  private  final SparkFlex kickMotor;

  public ShooterSubsystem() {
    finalShootMotor = new SparkFlex(0, MotorType.kBrushless);
    motorFeedforward = new SimpleMotorFeedforward(1, 3, 2);
    finalShootPID = new PIDController(0, 0, 0);
    kickMotor = new SparkFlex(0, MotorType.kBrushless);
    SmartDashboard.putNumber("Velocity", 0);
    SmartDashboard.putNumber("Acceleration", 0);
    shootEncoder = finalShootMotor.getAbsoluteEncoder();
  }

  //Shoot balls. None adjustable velocity
  public void Shoot() {
    finalShootMotor.setVoltage(
        motorFeedforward.calculate(
                SmartDashboard.getNumber("Velocity", 0),
                SmartDashboard.getNumber("Acceleration", 0))
            + finalShootPID.calculate(
                shootEncoder.getVelocity(), SmartDashboard.getNumber("Velocity", 0)));
  }

  //Used to kick the balls up from the storage up into the shooter
  public void KickBalls() {
    if (shootEncoder.getVelocity() == SmartDashboard.getNumber("Velocity", 0)) {
      kickMotor.set(1);
    } else{
      Shoot();
    }
  }

  public void StopShoot() {
    finalShootMotor.set(0);
  }
}
