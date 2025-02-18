package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandSubsystem;

public class CoralIn extends Command{
    
    public CoralIn(){
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        HandSubsystem.CoralIn();
    }

    @Override
    public void end(boolean interrupted){
        HandSubsystem.CoralStop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }


    


}
