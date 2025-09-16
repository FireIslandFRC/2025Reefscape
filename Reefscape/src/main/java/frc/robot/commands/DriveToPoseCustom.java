
package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

@Logged
public class DriveToPoseCustom extends Command {
  private SwerveSubsystem swerveSubs;

  private double speedX, speedTurn, speedY;
  private boolean done;
  private PIDController PIDControllerTurn, PIDControllerY, PIDControllerX;
  private Pose2d goalPose;
  public final static Joystick D_CONTROLLER = new Joystick(ControllerConstants.kDriverControllerPort);
  private final JoystickButton engageTargetAuto = new JoystickButton(D_CONTROLLER, 3);

  /* * * CONSTRUCTOR * * */
  /*
   * @param swerveSubs the swerve subsystem
   * 
   * @param xSupplier value input for strafe on x-axis
   * 
   * @param ySupplier value input for strafe on y-axis
   * 
   * @param zSupplier value input for rotation
   * 
   * @param fieldOriented whether or not we want the bot to run in field oriented
   */
  public DriveToPoseCustom( Pose2d goalPose, SwerveSubsystem swerveSubs ) {
    this.swerveSubs = swerveSubs;
    this.goalPose = goalPose;
    PIDControllerX = new PIDController(1, 0, 0);
    PIDControllerY = new PIDController(1, 0, 0);
    PIDControllerTurn = new PIDController(0.05, 0, 0);
    PIDControllerTurn.enableContinuousInput(-180, 180);
    done = false;
    // PIDControllerX.setTolerance(0.25);
    //addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    speedX = PIDControllerX.calculate(swerveSubs.getPose().getX(), goalPose.getX());
    speedY = PIDControllerY.calculate(swerveSubs.getPose().getY(), goalPose.getY());
    speedTurn = PIDControllerTurn.calculate(swerveSubs.getPose().getRotation().getDegrees(), goalPose.getRotation().getDegrees());

    SmartDashboard.putNumber("X goal", goalPose.getX());
    SmartDashboard.putNumber("Y goal", goalPose.getY());
    SmartDashboard.putNumber("X current", swerveSubs.getPose().getX());
    SmartDashboard.putNumber("Y current", swerveSubs.getPose().getY());

    if(engageTargetAuto.getAsBoolean()){
      addRequirements(swerveSubs);
      swerveSubs.drive(speedX, speedY, speedTurn, true, 0.8);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubs.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    engageTargetAuto.onFalse(new InstantCommand(() -> done = true));
    return done;
  }

  /* * * ADDED METHODS * * */
  public double deadzone(double num) {
    return Math.abs(num) > 0.03 ? num : 0; // CHECKME test optimal offset
  }

  private static double modifyAxis(double num) {
    // Square the axis
    num = Math.copySign(num * num, num);

    return num;
  }

  public void periodic() {

  }
}
