package frc.robot.commands.processor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.ProcessorSubsystem;

public class ProcessorDeposit extends Command{
    
    private final ProcessorSubsystem processorSubsystem;
    private double angle;
    private Timer timer;

    public ProcessorDeposit(ProcessorSubsystem processorSubsystem, double angle){
         this.processorSubsystem = processorSubsystem;
         this.angle = angle;
         timer = new Timer();
    }

    @Override
    public void initialize(){
        timer.restart();
    }

    @Override
    public void execute(){
        processorSubsystem.processorWheelOut();
        if (timer.get() > 0.4){
            processorSubsystem.processorToAngle(angle);
        }
        System.out.println(timer.get());
    }

    @Override
    public void end(boolean interrupted){
        processorSubsystem.processorPivotStop();
        processorSubsystem.processorWheelStop();
        timer.restart();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
