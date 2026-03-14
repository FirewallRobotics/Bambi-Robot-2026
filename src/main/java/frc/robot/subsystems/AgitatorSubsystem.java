package frc.robot.subsystems;

import java.lang.ProcessBuilder.Redirect.Type;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AgitatorSubsystem extends SubsystemBase{

    private SparkFlex agitatorMotor;
    private SparkFlexConfig agitatorConfig;
    private SparkClosedLoopController agitatorClosedLoop;
    private double velocity;

    public AgitatorSubsystem(){
        agitatorMotor = new SparkFlex(25, MotorType.kBrushless);
        agitatorConfig = new SparkFlexConfig();
        agitatorClosedLoop = agitatorMotor.getClosedLoopController();

        agitatorConfig.closedLoop
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
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        agitatorMotor.configure(agitatorConfig,
            com.revrobotics.ResetMode.kNoResetSafeParameters,
            com.revrobotics.PersistMode.kPersistParameters);
        velocity = 1000;
    }

    public void StartAgitator(){
        agitatorClosedLoop.setSetpoint(velocity, ControlType.kVelocity);
    }

    public void StopAgitator(){
        agitatorMotor.setVoltage(0);
    }


}
