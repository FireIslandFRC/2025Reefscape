package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TargetLocationConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


public class updateTeleTargetCommand extends Command{
    
    private final SwerveSubsystem swerveSubsystem;
    private PathConstraints constraints;
    static Pose2d currentTarget = TargetLocationConstants.slicePose1;


    public updateTeleTargetCommand(SwerveSubsystem subsystem, Pose2d target){
        swerveSubsystem = subsystem;
        currentTarget = target;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        
    // Create the constraints to use while pathfinding
    constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    }

    @Override
    public void execute(){

    }

    public void setTargetPose(Pose2d target){
        currentTarget = target;
        SmartDashboard.putString("target", currentTarget.toString());
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
