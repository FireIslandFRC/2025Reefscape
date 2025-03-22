package frc.robot.commands.processor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.ProcessorSubsystem;

public class ProcessorPivotDownCommand extends Command{

    private final ProcessorSubsystem processorSubsystem;
    
    public ProcessorPivotDownCommand(ProcessorSubsystem processorSubsystem){
        this.processorSubsystem = processorSubsystem;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        processorSubsystem.processorPivotDown();
    }

    @Override
    public void end(boolean interrupted){
        processorSubsystem.processorPivotStop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
