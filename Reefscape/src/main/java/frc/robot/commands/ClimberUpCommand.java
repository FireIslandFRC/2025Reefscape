package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandSubsystem;

public class ClimberUpCommand extends Command{
    
    private final HandSubsystem climberSubsystem;

    public ClimberUpCommand(HandSubsystem subsystem){
        climberSubsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        climberSubsystem.climbUp();
    }

    @Override
    public void end(boolean interrupted){
        climberSubsystem.ClimbStop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }


    


}
