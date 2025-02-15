package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandSubsystem;

public class CoralOut extends Command{
    
    public CoralOut(){
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        HandSubsystem.CoralOut();
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
