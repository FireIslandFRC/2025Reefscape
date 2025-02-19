package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.S_DriveCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer extends SubsystemBase{
  
  private final SwerveSubsystem swerveSubs = new SwerveSubsystem();

  public final static XboxController drive = new XboxController(ControllerConstants.kDriverControllerPort);

  //NOTE add button ids to Constants?
  //DRIVE BUTTONS     
  private final JoystickButton speedButton = new JoystickButton(drive, 1);
  private final JoystickButton fieldOriented = new JoystickButton(drive, 2);
  private final JoystickButton resetPigeonButton = new JoystickButton(drive, 3); //FIXME add back in
  private final JoystickButton lockbutton = new JoystickButton(drive, 3); //FIXME add back in

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    swerveSubs.setDefaultCommand(
      new S_DriveCommand(
        swerveSubs,
        () -> -drive.getLeftY(), 
        () -> -drive.getLeftX(), 
        () -> -drive.getRightX(), 
        () -> fieldOriented.getAsBoolean(), 
        () -> speedButton.getAsBoolean()
      )
    );
    
     boolean isCompetition = false;

    // Build an auto chooser. This will use Commands.none() as the default option.
    // As an example, this will only show autos that start with "comp" while at
    // competition as defined by the programmer
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );

    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings

    Shuffleboard.getTab("Fi")
   .add("Target", "r_s2")
   .withWidget("Field Vie") // specify the widget here
   .getEntry();

    configureBindings();
  }

  private void configureBindings() {
    resetPigeonButton.onTrue(new InstantCommand(() -> swerveSubs.resetPigeon()));
    //TODO: all buttons configurations
    //lockbutton.whileTrue(lockCommand().andThen( new PrintCommand("X Button Working")));
  }

  protected Command lockCommand() {
    return this.runOnce(() -> swerveSubs.lock());
  }
  
  protected Command straightenCommand() {
    return this.runOnce(() -> swerveSubs.straightenWheels());
  }

  public Command getAutonomousCommand() {
    //An example command will be run in autonomous
    return autoChooser.getSelected();

  }

  @Override
  public void periodic() {
    
  }

}