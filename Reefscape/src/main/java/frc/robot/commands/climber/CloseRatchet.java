package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class CloseRatchet extends Command{
    
    private final ClimberSubsystem climberSubsystem;

    public CloseRatchet(ClimberSubsystem subsystem){
        climberSubsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){

        climberSubsystem.closedRatchet();
        System.out.println("Closed");
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
