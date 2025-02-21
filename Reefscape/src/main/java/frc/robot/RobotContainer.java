package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.S_DriveCommand;

import java.lang.ModuleLayer.Controller;

import javax.print.attribute.standard.JobHoldUntil;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
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

  public final static Joystick D_CONTROLLER = new Joystick(ControllerConstants.kDriverControllerPort);
  public final static Joystick OP_CONTROLLER = new Joystick(ControllerConstants.kOperatorControllerPort);

  //OP BUTTONS 
  private final JoystickButton armLevel1 = new JoystickButton(OP_CONTROLLER, 5);
  private final JoystickButton armLevel2 = new JoystickButton(OP_CONTROLLER, 6);
  private final JoystickButton armLevel3 = new JoystickButton(OP_CONTROLLER, 7); 
  private final JoystickButton armLoading = new JoystickButton(OP_CONTROLLER, 8);
  private final JoystickButton armManualUp = new JoystickButton(OP_CONTROLLER, 10);
  private final JoystickButton armManualDown = new JoystickButton(OP_CONTROLLER, 9);

  private final JoystickButton intake = new JoystickButton(OP_CONTROLLER, 2);
  private final JoystickButton outtake = new JoystickButton(OP_CONTROLLER, 1);  
  
  private final JoystickButton targetSlice1 = null;
  private final JoystickButton targetSlice2 = null;
  private final JoystickButton targetSlice3 = null;
  private final JoystickButton targetSlice4 = null;
  private final JoystickButton targetSlice5 = null;
  private final JoystickButton targetSlice6 = null;

  private final JoystickButton targetLoading1 = new JoystickButton(OP_CONTROLLER, 3); // FIXME side dependant
  private final JoystickButton targetLoading2 = new JoystickButton(OP_CONTROLLER, 4); 

  private final JoystickButton targetCage1 = new JoystickButton(OP_CONTROLLER, 13); 
  private final JoystickButton targetCage2 = new JoystickButton(OP_CONTROLLER, 12);
  private final JoystickButton targetCage3 = new JoystickButton(OP_CONTROLLER, 11); 

  private final GenericHID m_stick = new GenericHID(ControllerConstants.kOperatorControllerPort);
  // private final JoystickButton openRatchet = new JoystickButton(drive, 1); FIXME uncoment for ratchet, needs button ids
  // private final JoystickButton closeRatchet = new JoystickButton(drive, 2);
  //private final JoystickButton Ground = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);
  //AXIS 
  //private final int joystickAxis = XboxController.Axis.kRightY.value;
  //public Field2d m_field;

  //NOTE add button ids to Constants?


  //DRIVE BUTTONS     
  private final JoystickButton speedSlow = new JoystickButton(D_CONTROLLER, 1);
  private final JoystickButton speedEmergency = new JoystickButton(D_CONTROLLER, 3);
  private final JoystickButton fieldOriented = new JoystickButton(D_CONTROLLER, 9);
  private final JoystickButton resetPigeonButton = new JoystickButton(D_CONTROLLER, 16);
  private final JoystickButton lockbutton = new JoystickButton(D_CONTROLLER, 10);
  private final JoystickButton targetSliceLeft = null;
  private final JoystickButton targetSliceRight = null;
  private final JoystickButton engageTargetAuto = new JoystickButton(D_CONTROLLER, 2);
  private final JoystickButton climbUp = new JoystickButton(D_CONTROLLER, 5);
  private final JoystickButton climbDown = new JoystickButton(D_CONTROLLER, 6);  
  private final JoystickButton ratchetEngage = new JoystickButton(D_CONTROLLER, 7);
  private final JoystickButton ratchetDisengage = new JoystickButton(D_CONTROLLER, 8);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    swerveSubs.setDefaultCommand(
      new S_DriveCommand(
        swerveSubs, // CHECKME possible flip of negative values
        () -> -D_CONTROLLER.getY(), 
        () -> -D_CONTROLLER.getX(), 
        () ->  D_CONTROLLER.getTwist(), 
        () -> fieldOriented.getAsBoolean(), 
        () -> speedSlow.getAsBoolean() // TODO add emergency speed
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

    // wristUp.whileTrue(new PivotUpCommand());
    // wristDown.whileTrue(new PivotDownCommand());
    // coralIn.whileTrue(new CoralIn());
    // coralOut.whileTrue(new CoralOut());

    // armUp.whileTrue(new ArmUpCommand());
    // armDown.whileTrue(new ArmDownCommand());

    // resetPigeonButton.onTrue(new InstantCommand(() -> swerveSubs.resetPigeon()));

    // climberUp.whileTrue(new ClimberUpCommand(climberSubs));
    // climberDown.whileTrue(new ClimberDownCommand(climberSubs));

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
    System.out.println(m_stick.getPOV()); //FIXME move to controlled declarations, and map
  }

}