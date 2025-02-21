package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.S_DriveCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
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
  private final JoystickButton climberUp = new JoystickButton(drive, 7);
  private final JoystickButton climberDown = new JoystickButton(drive, 8);

  // private final JoystickButton openRatchet = new JoystickButton(drive, 1); FIXME uncoment for ratchet, needs button ids
  // private final JoystickButton closeRatchet = new JoystickButton(drive, 2);
  //private final JoystickButton Ground = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);
  //AXIS 
  //private final int joystickAxis = XboxController.Axis.kRightY.value;
  //public Field2d m_field;

  //NOTE add button ids to Constants?
  //DRIVE BUTTONS     
  private final JoystickButton speedButton = new JoystickButton(drive, 11);
  private final JoystickButton fieldOriented = new JoystickButton(drive, 9);
  private final JoystickButton resetPigeonButton = new JoystickButton(drive, 10); //FIXME add back in
  private final JoystickButton lockbutton = new JoystickButton(drive, 3); //FIXME add back in

  private final SendableChooser<Command> autoChooser;

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





    // SmartDashboard.putData("Auto Target", new Sendable() {
    //   @Override
    //   public void initSendable(SendableBuilder builder) {
    //     builder.setSmartDashboardType("FieldVisualWidget");

    //     builder.addStringProperty("setTarget", () -> null, null);
    //   }
    // });





    Shuffleboard.getTab("Fi")
   .add("Target", "r_s2")
   .withWidget("Field Vie") // specify the widget here
   .getEntry();

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

    climberUp.whileTrue(new ClimberUpCommand(climberSubs));
    climberDown.whileTrue(new ClimberDownCommand(climberSubs));

    //TODO: all buttons
    //lockbutton.whileTrue(lockCommand().andThen( new PrintCommand("X Button Working")));

    // closeRatchet.onTrue(new CloseRatchet(climberSubs)); FIXME 
    // openRatchet.onTrue(new OpenRatchet(climberSubs));
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