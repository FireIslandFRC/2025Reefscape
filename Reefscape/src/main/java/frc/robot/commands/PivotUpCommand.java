package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandSubsystem;

public class PivotUpCommand extends Command{
    
    private final HandSubsystem handSubsystem;

    public PivotUpCommand(){
        handSubsystem = new HandSubsystem();

        addRequirements(handSubsystem);
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
