package frc.robot.commands.processor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.ProcessorSubsystem;

public class ProcessorPickUp extends Command{
    
    private final ProcessorSubsystem processorSubsystem;
    private double angle;

    public ProcessorPickUp(ProcessorSubsystem processorSubsystem, double angle){
         this.processorSubsystem = processorSubsystem;
         this.angle = angle;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        processorSubsystem.processorToAngle(angle);
        processorSubsystem.processorWheelIn();
    }

    @Override
    public void end(boolean interrupted){
        processorSubsystem.processorPivotStop();
        processorSubsystem.processorWheelStop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
