package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandSubsystem;

public class PivotToAngle extends Command{
    
    private final HandSubsystem handSubsystem;
    private double angle;

    public PivotToAngle(HandSubsystem handSubsystem, double angle){
         this.handSubsystem = handSubsystem;
         this.angle = angle;

        // addRequirements(handSubsystem);
    }

    @Override
    public void initialize(){}

    @SuppressWarnings("static-access")
    @Override
    public void execute(){
        handSubsystem.WristToAngle(angle);
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
