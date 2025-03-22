package frc.robot.commands.processor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.ProcessorSubsystem;

public class AlgaeOut extends Command{
    
    private final ProcessorSubsystem processorSubsystem;

    public AlgaeOut(ProcessorSubsystem processorSubsystem){
        this.processorSubsystem = processorSubsystem;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        processorSubsystem.processorWheelOut();
    }

    @Override
    public void end(boolean interrupted){
        processorSubsystem.processorWheelStop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
