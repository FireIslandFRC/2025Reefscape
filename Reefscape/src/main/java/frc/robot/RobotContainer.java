package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TargetLocationConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.climber.ClimberDownCommand;
import frc.robot.commands.PathToPose;
import frc.robot.commands.S_DriveCommand;
import frc.robot.commands.updateTeleTargetCommand;
import frc.robot.commands.arm.ArmDownCommand;
import frc.robot.commands.arm.ArmSetPositionCommand;
import frc.robot.commands.arm.ArmUpCommand;
import frc.robot.commands.climber.ClimberUpCommand;
import frc.robot.commands.climber.CloseRatchet;
import frc.robot.commands.climber.OpenRatchet;
import frc.robot.commands.endEffector.CoralIn;
import frc.robot.commands.endEffector.CoralOut;
import frc.robot.commands.endEffector.PivotDownCommand;
import frc.robot.commands.endEffector.PivotUpCommand;

import java.lang.ModuleLayer.Controller;

import javax.print.attribute.standard.JobHoldUntil;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.S_DriveCommand;

public class RobotContainer extends SubsystemBase{
  
  private final SwerveSubsystem swerveSubs = new SwerveSubsystem();
  private final ClimberSubsystem climberSubs = new ClimberSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem(); 


  public static int opSliceTarget = 1;

  static String currentTarget = "default";

  // private updateTeleTargetCommand updateTeleTargetCommand = new updateTeleTargetCommand(swerveSubs);


  public final static Joystick D_CONTROLLER = new Joystick(ControllerConstants.kDriverControllerPort);
  private final static Joystick OP_CONTROLLER = new Joystick(ControllerConstants.kOperatorControllerPort);

  //OP BUTTONS 
  private final JoystickButton armLevel1 = new JoystickButton(OP_CONTROLLER, 5);
  private final JoystickButton armLevel2 = new JoystickButton(OP_CONTROLLER, 6);
  private final JoystickButton armLevel3 = new JoystickButton(OP_CONTROLLER, 7); 
  private final JoystickButton armLoading = new JoystickButton(OP_CONTROLLER, 8);
  private final JoystickButton armManualUp = new JoystickButton(OP_CONTROLLER, 10);
  private final JoystickButton armManualDown = new JoystickButton(OP_CONTROLLER, 9);

  private final JoystickButton endEffectorIntake = new JoystickButton(OP_CONTROLLER, 2);
  private final JoystickButton endEffectorOuttake = new JoystickButton(OP_CONTROLLER, 1);
  
  private final POVButton targetSlice1 = new POVButton(OP_CONTROLLER, 0);
  private final POVButton targetSlice2 = new POVButton(OP_CONTROLLER, 45);
  private final POVButton targetSlice3 = new POVButton(OP_CONTROLLER, 135);
  private final POVButton targetSlice4 = new POVButton(OP_CONTROLLER, 180);
  private final POVButton targetSlice5 = new POVButton(OP_CONTROLLER, 225);
  private final POVButton targetSlice6 = new POVButton(OP_CONTROLLER, 315);

  private final JoystickButton targetCoralLoading1 = new JoystickButton(OP_CONTROLLER, 3); // FIXME side dependant
  private final JoystickButton targetCoralLoading2 = new JoystickButton(OP_CONTROLLER, 4); 

  private final JoystickButton targetCage1 = new JoystickButton(OP_CONTROLLER, 13); 
  private final JoystickButton targetCage2 = new JoystickButton(OP_CONTROLLER, 12);
  private final JoystickButton targetCage3 = new JoystickButton(OP_CONTROLLER, 11); 

  private final JoystickButton wristUp = new JoystickButton(OP_CONTROLLER, 14); 
  private final JoystickButton wristDown = new JoystickButton(OP_CONTROLLER, 15); 

  private final GenericHID m_stick = new GenericHID(ControllerConstants.kOperatorControllerPort);
  //AXIS 
  //private final int joystickAxis = XboxController.Axis.kRightY.value;
  //public Field2d m_field;

  //NOTE add button ids to Constants? 

  //DRIVE BUTTONS     
  private final JoystickButton speedSlow = new JoystickButton(D_CONTROLLER, 1);
  private final JoystickButton speedEmergency = new JoystickButton(D_CONTROLLER, 3);
  private final JoystickButton fieldOriented = new JoystickButton(D_CONTROLLER, 9);
  private final JoystickButton resetPigeonButton = new JoystickButton(D_CONTROLLER, 16);
  private final JoystickButton lockbutton = new JoystickButton(D_CONTROLLER, 10); //FIXME
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
        () -> D_CONTROLLER.getTwist(), 
        () -> fieldOriented.getAsBoolean(), 
        () -> speedSlow.getAsBoolean(),
        () -> speedEmergency.getAsBoolean() 
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



    Shuffleboard.getTab("Comp")
    .add("TargetSelect", currentTarget)
    .withWidget("FieldVisualWidget") // specify the widget here
    .getEntry();

    configureBindings();
  }

  private void configureBindings() {

    // wristUp.whileTrue(new PivotUpCommand());
    // wristDown.whileTrue(new PivotDownCommand());
    endEffectorIntake.whileTrue(new CoralIn());
    endEffectorOuttake.whileTrue(new CoralOut());
    armLoading.whileTrue(new ArmSetPositionCommand(0));
    armLevel1.whileTrue(new ArmSetPositionCommand(1));
    armLevel2.whileTrue(new ArmSetPositionCommand(2));
    armLevel3.whileTrue(new ArmSetPositionCommand(3));
    armManualUp.whileTrue(new ArmUpCommand());
    armManualDown.whileTrue(new ArmDownCommand());

    resetPigeonButton.onTrue(new InstantCommand(() -> swerveSubs.resetPigeon()));

    climbUp.whileTrue(new ClimberUpCommand(climberSubs));
    climbDown.whileTrue(new ClimberDownCommand(climberSubs));

    //TODO: all buttons
    //lockbutton.whileTrue(lockCommand().andThen( new PrintCommand("X Button Working")));

    ratchetEngage.onTrue(new CloseRatchet(climberSubs)); //FIXME set positions
    ratchetDisengage.onTrue(new OpenRatchet(climberSubs));

    // targetSlice1.onTrue(new InstantCommand(() -> currentTarget = TargetLocationConstants.slicePose1));
    // targetSlice2.onTrue(new InstantCommand(() -> currentTarget = TargetLocationConstants.slicePose2));
    // targetSlice3.onTrue(new InstantCommand(() -> currentTarget = TargetLocationConstants.slicePose3));
    // targetSlice4.onTrue(new InstantCommand(() -> currentTarget = TargetLocationConstants.slicePose4));
    // targetSlice5.onTrue(new InstantCommand(() -> currentTarget = TargetLocationConstants.slicePose5));
    // targetSlice6.onTrue(new InstantCommand(() -> currentTarget = TargetLocationConstants.slicePose6));

    // targetSlice1.onTrue(new InstantCommand(() -> swerveSubs.telePath(TargetLocationConstants.slicePose1, engageTargetAuto)));
    // targetSlice2.onTrue(new InstantCommand(() ->  swerveSubs.telePath(TargetLocationConstants.slicePose2, engageTargetAuto)));
    // targetSlice3.onTrue(new InstantCommand(() ->  swerveSubs.telePath(TargetLocationConstants.slicePose3, engageTargetAuto)));
    // targetSlice4.onTrue(new InstantCommand(() ->  swerveSubs.telePath(TargetLocationConstants.slicePose4, engageTargetAuto)));
    // targetSlice5.onTrue(new InstantCommand(() ->  swerveSubs.telePath(TargetLocationConstants.slicePose5, engageTargetAuto)));
    // targetSlice6.onTrue(new InstantCommand(() ->  swerveSubs.telePath(TargetLocationConstants.slicePose6, engageTargetAuto)));
    
    targetSlice1.onTrue(new PathToPose(TargetLocationConstants.slicePose1, swerveSubs)).onTrue(new InstantCommand(() -> currentTarget = Robot.color + "_s1"));  //FIXME end after other button pressed
    targetSlice2.onTrue(new PathToPose(TargetLocationConstants.slicePose2, swerveSubs)).onTrue(new InstantCommand(() -> currentTarget = Robot.color + "_s2"));
    targetSlice3.onTrue(new PathToPose(TargetLocationConstants.slicePose3, swerveSubs)).onTrue(new InstantCommand(() -> currentTarget = Robot.color + "_s3"));
    targetSlice4.onTrue(new PathToPose(TargetLocationConstants.slicePose4, swerveSubs)).onTrue(new InstantCommand(() -> currentTarget = Robot.color + "_s4"));
    targetSlice5.onTrue(new PathToPose(TargetLocationConstants.slicePose5, swerveSubs)).onTrue(new InstantCommand(() -> currentTarget = Robot.color + "_s5"));
    targetSlice6.onTrue(new PathToPose(TargetLocationConstants.slicePose6, swerveSubs)).onTrue(new InstantCommand(() -> currentTarget = Robot.color + "_s6"));

    targetCoralLoading1.onTrue(new PathToPose(TargetLocationConstants.coralLoad1, swerveSubs)).onTrue(new InstantCommand(() -> currentTarget = Robot.color + "_cl1"));
    targetCoralLoading2.onTrue(new PathToPose(TargetLocationConstants.coralLoad2, swerveSubs)).onTrue(new InstantCommand(() -> currentTarget = Robot.color + "_cl2"));
    
    targetCage1.onTrue(new PathToPose(TargetLocationConstants.cage1, swerveSubs)).onTrue(new InstantCommand(() -> currentTarget = Robot.color + "_cg1"));
    targetCage1.onTrue(new PathToPose(TargetLocationConstants.cage2, swerveSubs)).onTrue(new InstantCommand(() -> currentTarget = Robot.color + "_cg2"));
    targetCage1.onTrue(new PathToPose(TargetLocationConstants.cage1, swerveSubs)).onTrue(new InstantCommand(() -> currentTarget = Robot.color + "_cg3"));
    
    wristUp.whileTrue(new PivotUpCommand());
    wristDown.whileTrue(new PivotDownCommand());
    // engageTargetAuto.whileTrue(swerveSubs.telePath(currentTarget));
    // engageTargetAuto.onTrue(swerveSubs.telePath(() -> currentTarget.getX(), () -> currentTarget.getY(), () -> currentTarget.getRotation().getDegrees()));
    // targetSlice1.and(engageTargetAuto.whileTrue(swerveSubs.telePath(TargetLocationConstants.slicePose1)));
    // targetSlice2.and(engageTargetAuto.whileTrue(swerveSubs.telePath(TargetLocationConstants.slicePose2)));
    // targetSlice3.and(engageTargetAuto.whileTrue(swerveSubs.telePath(TargetLocationConstants.slicePose3)));
    // targetSlice4.and(engageTargetAuto.whileTrue(swerveSubs.telePath(TargetLocationConstants.slicePose4)));
    // targetSlice5.and(engageTargetAuto.whileTrue(swerveSubs.telePath(TargetLocationConstants.slicePose5)));
    // targetSlice6.and(engageTargetAuto.whileTrue(swerveSubs.telePath(TargetLocationConstants.slicePose6)));

    // engageTargetAuto.whileTrue(swerveSubs.telePath());

  }

  // private void setTargetPose(){
  //   int opSliceTarget = m_stick.getPOV();

  //   switch (opSliceTarget) {
  //    case -1:
  //      break;
  //    case 0:
  //     //  updateTeleTargetCommand.setTargetPose(TargetLocationConstants.slicePose1);
  //      currentTarget = TargetLocationConstants.slicePose1;
  //      targetLabel = 1; 
  //      break;
  //    case 45:
  //    currentTarget = TargetLocationConstants.slicePose2;
  //    targetLabel = 2; 
  //      break;
  //    case 90:
  //      break;
  //    case 135:
  //    currentTarget = TargetLocationConstants.slicePose3;
  //    targetLabel = 3; 
  //      break;
  //    case 180:
  //    currentTarget = TargetLocationConstants.slicePose4;
  //    targetLabel = 4; 
  //      break;
  //    case 225:
  //    currentTarget = TargetLocationConstants.slicePose5;
  //    targetLabel = 5; 
  //      break;
  //    case 270:
  //      break;
  //    case 315:
  //    currentTarget = TargetLocationConstants.slicePose6;
  //    targetLabel = 6; 
  //      break;
  //    default:
  //      break;
  //   }

  //   SmartDashboard.putString("targetSlice", "slice" + targetLabel);
  //   SmartDashboard.putString("target", currentTarget.toString());

  // }

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
     //System.out.println(m_stick.getPOV());
     //setTargetPose();

     //opSliceTarget = m_stick.getPOV();
     SmartDashboard.putString("targetSlice", "slice " + currentTarget);
     SmartDashboard.putString("TargetSelect", currentTarget);
     SmartDashboard.putString("color", Robot.color);
  }

}