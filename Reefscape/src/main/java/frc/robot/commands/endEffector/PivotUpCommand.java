package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandSubsystem;

public class PivotUpCommand extends Command{
    
    private final HandSubsystem handSubsystem;

    public PivotUpCommand(HandSubsystem handSubsystem){

        this.handSubsystem = handSubsystem;
        
        // addRequirements(handSubsystem);
    }

    @Override
    public void initialize(){}

    @SuppressWarnings("static-access")
    @Override
    public void execute(){
        handSubsystem.PivotUp();
    }

    @SuppressWarnings("static-access")
    @Override
    public void end(boolean interrupted){
        handSubsystem.PivotStop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }


    


}
