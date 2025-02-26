package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class PathToPose extends Command {

    public final static Joystick D_CONTROLLER = new Joystick(ControllerConstants.kDriverControllerPort);
    public final static Joystick OP_CONTROLLER = new Joystick(ControllerConstants.kOperatorControllerPort);
    private Pose2d pathfindingPose;
    Command pathFindCommand;

    private final JoystickButton engageTargetAuto = new JoystickButton(D_CONTROLLER, 2);
    // private final POVButton CancelAuto1 = new POVButton(D_CONTROLLER, 0);
    // private final POVButton CancelAuto2 = new POVButton(D_CONTROLLER, 45);
    // private final POVButton CancelAuto3 = new POVButton(D_CONTROLLER, 135);
    // private final POVButton CancelAuto4 = new POVButton(D_CONTROLLER, 180);
    // private final POVButton CancelAuto5 = new POVButton(D_CONTROLLER, 225);
    // private final POVButton CancelAuto6 = new POVButton(D_CONTROLLER, 315);


    /*
     * Pathfinds to a specific pose given
     * @param pathfindingPose The pose to pathfind to
     */
    public PathToPose(Pose2d pathfindingPose, SwerveSubsystem drivetrain) {
        this.pathfindingPose = pathfindingPose;

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
        engageTargetAuto.whileTrue(pathFindCommand);
        // if (CancelAuto1.getAsBoolean() || CancelAuto1.getAsBoolean() || CancelAuto1.getAsBoolean() || CancelAuto1.getAsBoolean() || CancelAuto1.getAsBoolean() || CancelAuto1.getAsBoolean()){
        //     end(true);
        // }
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