package frc.robot.commands.processor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.ProcessorSubsystem;

public class ProcessorPivotUpCommand extends Command{

    private final ProcessorSubsystem processorSubsystem;
    
    public ProcessorPivotUpCommand(ProcessorSubsystem processorSubsystem){
        this.processorSubsystem = processorSubsystem;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        processorSubsystem.processorPivotUp();
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
