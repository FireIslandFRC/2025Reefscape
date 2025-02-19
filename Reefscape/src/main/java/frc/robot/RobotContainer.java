package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.S_DriveCommand;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.CoralIn;
import frc.robot.commands.CoralOut;
import frc.robot.commands.PivotDownCommand;
import frc.robot.commands.PivotUpCommand;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.S_DriveCommand;
import frc.robot.commands.CloseRatchet;
import frc.robot.commands.OpenRatchet;

public class RobotContainer extends SubsystemBase{
  
  private final SwerveSubsystem swerveSubs = new SwerveSubsystem();
  private final ClimberSubsystem climberSubs = new ClimberSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem(); 

  public final static XboxController drive = new XboxController(ControllerConstants.kDriverControllerPort);

    //private final XboxController drive = new XboxController(0);

  //DRIVE BUTTONS 
  private final JoystickButton armUp = new JoystickButton(drive, 1);
  private final JoystickButton armDown = new JoystickButton(drive, 2);
  private final JoystickButton coralIn = new JoystickButton(drive, 3); 
  private final JoystickButton coralOut = new JoystickButton(drive, 4);
  private final JoystickButton wristUp = new JoystickButton(drive, 5);
  private final JoystickButton wristDown = new JoystickButton(drive, 6);
  private final JoystickButton openRatchet = new JoystickButton(drive, 1);
  private final JoystickButton closeRatchet = new JoystickButton(drive, 2);
  //private final JoystickButton Ground = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);
  //AXIS 
  //private final int joystickAxis = XboxController.Axis.kRightY.value;
  //public Field2d m_field;

  //NOTE add button ids to Constants?
  //DRIVE BUTTONS     
  private final JoystickButton speedButton = new JoystickButton(drive, 8);
  private final JoystickButton fieldOriented = new JoystickButton(drive, 9);
  private final JoystickButton resetPigeonButton = new JoystickButton(drive, 10); //FIXME add back in
  private final JoystickButton lockbutton = new JoystickButton(drive, 3); //FIXME add back in

  public RobotContainer() {
    swerveSubs.setDefaultCommand(
      new S_DriveCommand(
        swerveSubs,
        () -> -drive.getLeftY(), 
        () -> -drive.getLeftX(), 
        () -> drive.getRightX(), 
        () -> fieldOriented.getAsBoolean(), 
        () -> speedButton.getAsBoolean()
      )
    );
    
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {

    wristUp.whileTrue(new PivotUpCommand());
    wristDown.whileTrue(new PivotDownCommand());
    coralIn.whileTrue(new CoralIn());
    coralOut.whileTrue(new CoralOut());


    armUp.whileTrue(new ArmUpCommand());
    armDown.whileTrue(new ArmDownCommand());

    resetPigeonButton.onTrue(new InstantCommand(() -> swerveSubs.resetPigeon()));

    //TODO: all buttons
    //lockbutton.whileTrue(lockCommand().andThen( new PrintCommand("X Button Working")));
    closeRatchet.onTrue(new CloseRatchet(climberSubs));
    openRatchet.onTrue(new OpenRatchet(climberSubs));
  }

  

  protected Command lockCommand() {
    return this.runOnce(() -> swerveSubs.lock());
  }
  
  protected Command straightenCommand() {
    return this.runOnce(() -> swerveSubs.straightenWheels());
  }

  /*public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
    // return AutoBuilder.followPath(path);

  }*/

  @Override
  public void periodic() {
    
  }

}