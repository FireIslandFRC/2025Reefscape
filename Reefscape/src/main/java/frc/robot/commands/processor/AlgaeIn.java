package frc.robot.commands.processor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.ProcessorSubsystem;

public class AlgaeIn extends Command{
    
    private final ProcessorSubsystem processorSubsystem;

    public AlgaeIn(ProcessorSubsystem processorSubsystem){
        this.processorSubsystem = processorSubsystem;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        processorSubsystem.processorWheelIn();
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
