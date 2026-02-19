package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
    SparkFlex climberMotorLeft = new SparkFlex(Constants.ClimberSubsystem.CLIMBER_MOTOR_PORT_LEFT, MotorType.kBrushless);
    SparkFlex climberMotorRight = new SparkFlex(Constants.ClimberSubsystem.CLIMBER_MOTOR_PORT_RIGHT, MotorType.kBrushless);
    SparkFlexConfig climberMotorLeftConfig = new SparkFlexConfig();
    SparkFlexConfig climberMotorRightConfig = new SparkFlexConfig();

    SparkClosedLoopController loopControllerLeft;
    SparkClosedLoopController loopControllerRight;

    public ClimberSubsystem() {
        // Initialize climber hardware here
        climberMotorLeftConfig.inverted(Constants.ClimberSubsystem.CLIMBER_MOTOR_LEFT_INVERTED);
        climberMotorRightConfig.inverted(Constants.ClimberSubsystem.CLIMBER_MOTOR_RIGHT_INVERTED);


        // SparkClosedLoopController loopControllerleft = climberMotorLeft.getClosedLoopController();
        // climberMotorRight.getClosedLoopController().setPID(Constants.ClimberSubsystem.CLIMBER_KP, Constants.ClimberSubsystem.CLIMBER_KI, Constants.ClimberSubsystem.CLIMBER_KD);
        loopControllerLeft = climberMotorLeft.getClosedLoopController();
        loopControllerRight = climberMotorRight.getClosedLoopController();

        
        climberMotorLeft.configure(climberMotorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climberMotorRight.configure(climberMotorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void extendClimber() {
        loopControllerLeft.setSetpoint(Constants.ClimberSubsystem.CLIMBER_EXTENDED_POSITION, ControlType.kPosition); // Set to desired position to extend
        loopControllerRight.setSetpoint(Constants.ClimberSubsystem.CLIMBER_EXTENDED_POSITION, ControlType.kPosition);
    }
    public void retractClimber() {
        loopControllerLeft.setSetpoint(Constants.ClimberSubsystem.CLIMBER_RETRACTED_POSITION, ControlType.kPosition); // Set to desired position to retract
        loopControllerRight.setSetpoint(Constants.ClimberSubsystem.CLIMBER_RETRACTED_POSITION, ControlType.kPosition);
    }
    public boolean isFinished(){
        return loopControllerLeft.isAtSetpoint();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // You can add code here to update the climber state or perform actions
    }

    // Add methods to control the climber, such as extend, retract, stop, etc.  
    
}
