package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.S_DriveCommand;

public class RobotContainer extends SubsystemBase{
  //SUBSYSTEMS 
  private final SwerveSubsystem swerveSubs = new SwerveSubsystem(); 

  //private final Arm ArmSubs = new Arm(); 

  //SENDABLECHOOSER


  //CONTROLLERS  
  public final static XboxController drive = new XboxController(ControllerConstants.kDriverControllerPort);

    //private final XboxController drive = new XboxController(0);

  //DRIVE BUTTONS 
  private final JoystickButton Up = new JoystickButton(drive, 1);
  private final JoystickButton Down = new JoystickButton(drive, 2);
  private final JoystickButton resetPigeonButton = new JoystickButton(drive, 3);
  private final JoystickButton lockbutton = new JoystickButton(drive, 3);
  //private final JoystickButton Ground = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);
  //AXIS 
  //private final int joystickAxis = XboxController.Axis.kRightY.value;
  //public Field2d m_field;


  public RobotContainer() {
    /*swerveSubs.setDefaultCommand(
      new S_DriveCommand(
        swerveSubs,
        () -> -drive.getLeftY(), 
        () -> -drive.getLeftX(), 
        () -> -drive.getRightX(), 
        () -> fieldOriented.getAsBoolean(), 
        () -> speedButton.getAsBoolean()
      )
    );*/

    ///m_field = new Field2d();
    //SmartDashboard.putData(m_field);

    


    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    Up.whileTrue(new ArmUpCommand());
    //TODO: all buttons
    //lockbutton.whileTrue(lockCommand().andThen( new PrintCommand("X Button Working")));
    
  }

  

  private Command lockCommand() {
    return this.runOnce(() -> swerveSubs.lock());
  }
  
  public Command straightenCommand() { //FIXME look into method permisions
    return this.runOnce(() -> swerveSubs.straightenWheels());
  }
  /*public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
    // return AutoBuilder.followPath(path);

  }*/

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Kp", Constants.SwerveConstants.KP_TURNING);
    // SmartDashboard.putNumber("Ki", Constants.SwerveConstants.KI_TURNING);
    // SmartDashboard.putNumber("Kd", Constants.SwerveConstants.KD_TURNING);
    //m_field.setRobotPose(swerveSubs.getPose());
  }

}