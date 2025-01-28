package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandSubsystem;

public class PivotDownCommand extends Command{
    
    public PivotDownCommand(){
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        HandSubsystem.PivotDown();
    }

    @Override
    public void end(boolean interrupted){
        HandSubsystem.PivotStop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }


    


}
