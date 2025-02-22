package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class OpenRatchet extends Command{
    
    private final ClimberSubsystem climberSubsystem;

    public OpenRatchet(ClimberSubsystem subsystem){
        climberSubsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        climberSubsystem.openRatchet();
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
