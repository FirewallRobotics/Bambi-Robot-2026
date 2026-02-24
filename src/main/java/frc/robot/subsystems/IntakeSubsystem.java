package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.IntakeSubsystemConstants;

public class IntakeSubsystem {

    private final SparkFlex intakeMotor;
    private final SparkFlexConfig intakeMotorConfig;
    private final SparkClosedLoopController intakeClosedLoop;

    public IntakeSubsystem(){
        intakeMotor = new SparkFlex(10, MotorType.kBrushless);
        intakeMotorConfig = new SparkFlexConfig();
        intakeClosedLoop = intakeMotor.getClosedLoopController();

        intakeMotorConfig.smartCurrentLimit(40);

        intakeMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed loop
            // slot, as it will default to slot 0.
            .p(IntakeSubsystemConstants.intakeP)
            .i(IntakeSubsystemConstants.intakeI)
            .d(0)
            .outputRange(-1, 1)
            // Set PID values for velocity control in slot 1
            .p(0, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
            .feedForward
            // kV is now in Volts, so we multiply by the nominal voltage (12V)
            .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

        intakeMotor.configure(
            intakeMotorConfig,
            com.revrobotics.ResetMode.kNoResetSafeParameters,
            com.revrobotics.PersistMode.kPersistParameters);
    }

    public void StartIntake(){
        intakeClosedLoop.setSetpoint(IntakeSubsystemConstants.intakeVelocity, ControlType.kVelocity);
    }

    public void StopIntake(){
        intakeMotor.setVoltage(0);
    }



}
