package frc.robot.subsystems;


import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{ 
    private final SparkFlex leader;
    private final SparkFlex follower;
    public ClimberSubsystem () {
        leader = new SparkFlex(0, MotorType.kBrushless);
        follower = new SparkFlex(0, MotorType.kBrushless);
    }
    public void starter (double speed) {
        leader.set(speed);
    }
    public void finisher () {
        leader.set(0);
    }
}
