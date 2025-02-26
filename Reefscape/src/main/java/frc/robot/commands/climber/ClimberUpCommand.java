package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberUpCommand extends Command{
    
    private final ClimberSubsystem climberSubsystem;

    public ClimberUpCommand(ClimberSubsystem subsystem){
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
        climberSubsystem.climbStop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
