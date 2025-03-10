package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandSubsystem;

public class PivotUpCommand extends Command{
    
    private final HandSubsystem handSubsystem;

    public PivotUpCommand(HandSubsystem handSubsystem){

        this.handSubsystem = handSubsystem;
        
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        handSubsystem.PivotUp();
    }

    @Override
    public void end(boolean interrupted){
        handSubsystem.PivotStop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
