package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDownCommand extends Command{
    
    private final ClimberSubsystem climberSubsystem;

    public ClimberDownCommand(ClimberSubsystem subsystem){
        climberSubsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        climberSubsystem.climbDown();
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
