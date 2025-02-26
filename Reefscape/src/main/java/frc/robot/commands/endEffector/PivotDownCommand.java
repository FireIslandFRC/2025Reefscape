package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandSubsystem;

public class PivotDownCommand extends Command{

    private final HandSubsystem handSubsystem;
    
    public PivotDownCommand(HandSubsystem handSubsystem){
        this.handSubsystem = handSubsystem;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        handSubsystem.PivotDown();
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
