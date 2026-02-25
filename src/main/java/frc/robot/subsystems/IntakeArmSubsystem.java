package frc.robot.subsystems;

import java.lang.ProcessBuilder.Redirect.Type;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystemConstants;

public class IntakeArmSubsystem extends SubsystemBase{
    private final SparkFlex armMotor;
    private final SparkFlexConfig armConfig;
    private final SparkClosedLoopController armClosedLoopController;

    private double upPosition;
    private double downPosition;

    public IntakeArmSubsystem(){
        armMotor = new SparkFlex(17, MotorType.kBrushless);
        armConfig = new SparkFlexConfig();
        armClosedLoopController = armMotor.getClosedLoopController();
        upPosition = 0;
        downPosition = 0;

        

        armConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed loop
            // slot, as it will default to slot 0.
            .p(0)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            // Set PID values for velocity control in slot 1
            .p(0, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
            .feedForward
            .kS(0)
            .kG(0)
            .kCos(0)
            .kCosRatio(0);
        

    }

    public void AngleArm(boolean goingDown){
        if(goingDown){
            armMotor.set(-0.1);
        } else{
            armMotor.set(0.1);
        }
    }

    public void StopAngle(){
        armMotor.setVoltage(0);
    }

    public void SetAngle(boolean isGoingUp){
        if(isGoingUp){
            armClosedLoopController.setSetpoint(upPosition, ControlType.kPosition);
        } else{
            armClosedLoopController.setSetpoint(downPosition, ControlType.kPosition);
        }
    }

}
