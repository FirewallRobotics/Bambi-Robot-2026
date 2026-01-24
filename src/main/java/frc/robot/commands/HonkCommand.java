package frc.robot.commands;

import java.util.Arrays;
import java.util.Collection;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.traits.CommonDevice;

import edu.wpi.first.wpilibj2.command.Command;

public class HonkCommand extends Command {

    private Orchestra honkOrchestra;

    public HonkCommand(String filepath){
        final CANBus kCANBus = CANBus.roboRIO();
        final TalonFX FLD = new TalonFX(1, kCANBus);
        final TalonFX FRD = new TalonFX(2, kCANBus);
        final TalonFX RLD = new TalonFX(3, kCANBus);
        final TalonFX RRD = new TalonFX(4, kCANBus);
        final TalonFX FLT = new TalonFX(5, kCANBus);
        final TalonFX FRT = new TalonFX(6, kCANBus);
        final TalonFX RLT = new TalonFX(7, kCANBus);
        final TalonFX RRT = new TalonFX(8, kCANBus);

        final Collection<CommonDevice> honkCollection = Arrays.asList(FLD, FRD, RLD, RRD, FLT, FRT, RLT, RRT);


        honkOrchestra = new Orchestra(honkCollection, filepath);
    }

    @Override
    public void initialize(){
        honkOrchestra.play();
    }

    @Override
    public void end(boolean interrupted){
        honkOrchestra.stop();
    }

    public boolean isFinished(){
        return !honkOrchestra.isPlaying();
    }
}
