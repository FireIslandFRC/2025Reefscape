package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class PathToPose extends Command {

    public final static Joystick D_CONTROLLER = new Joystick(ControllerConstants.kDriverControllerPort);
    private SwerveSubsystem drivetrain;
    private PIDController headingController = new PIDController(0.1, 0, 0);
    private Pose2d pathfindingPose;
    Command pathFindCommand;

    private final JoystickButton engageTargetAuto = new JoystickButton(D_CONTROLLER, 2);


    /*
     * Pathfinds to a specific pose given
     * @param pathfindingPose The pose to pathfind to
     */
    public PathToPose(Pose2d pathfindingPose, SwerveSubsystem drivetrain) {
        this.pathfindingPose = pathfindingPose;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        pathFindCommand = AutoBuilder.pathfindToPose(
                pathfindingPose, new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720)));
    }

    @Override
    public void execute() {
        /*if (engageTargetAuto.getAsBoolean() && !pathFindCommand.isScheduled()){
            //pathFindCommand.schedule();
            pathFindCommand.until(() -> !engageTargetAuto.getAsBoolean());
            
        }*/
        engageTargetAuto.whileTrue(pathFindCommand);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            pathFindCommand.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return pathFindCommand.isFinished();
    }
}