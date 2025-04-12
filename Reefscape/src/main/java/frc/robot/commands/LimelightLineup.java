// package frc.robot.commands;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.LimelightHelpers;
// import frc.robot.subsystems.SwerveSubsystem;

// public class LimelightLineup extends Command {
//   private SwerveSubsystem swerveSubs;

//   private DoubleSupplier xSupplier, ySupplier, zSupplier;
//   private BooleanSupplier fieldOriented;
//   private double speedX, speedTurn;
//   private PIDController PIDControllerX;
//   private PIDController PIDControllerTurn;

//   /* * * CONSTRUCTOR * * */
//   /*
//    * @param swerveSubs the swerve subsystem
//    * 
//    * @param xSupplier value input for strafe on x-axis
//    * 
//    * @param ySupplier value input for strafe on y-axis
//    * 
//    * @param zSupplier value input for rotation
//    * 
//    * @param fieldOriented whether or not we want the bot to run in field oriented
//    */
//   public LimelightLineup(SwerveSubsystem swerveSubs, DoubleSupplier xSupplier, DoubleSupplier zSupplier) {
//     this.swerveSubs = swerveSubs;
//     this.xSupplier = xSupplier;
//     this.ySupplier = ySupplier;
//     this.zSupplier = zSupplier;
//     this.fieldOriented = fieldOriented;
//     PIDControllerTurn = new PIDController(0.01, 0, 0);
//     PIDControllerX = new PIDController(1, 0, 0);
//     PIDControllerTurn.enableContinuousInput(-180, 180);
//     addRequirements(swerveSubs);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     /* * * ALTERING VALUES * * */
//     // Joystick values -> double
//     double forwardSpeed = xSupplier.getAsDouble();
//     double zSpeed = zSupplier.getAsDouble();

//     int tagID = (int) LimelightHelpers.getFiducialID("limelight");
//     double rotationSetpoint;

//     switch (tagID) {
//       case 17:
//         rotationSetpoint = 60;
//         break;
//       case 18:
//         rotationSetpoint = 0;
//         break;
//       case 19:
//         rotationSetpoint = -60;
//         break;
//       case 20:
//         rotationSetpoint = -120;
//         break;
//       case 21:
//         rotationSetpoint = 180;
//         break;
//       case 22:
//         rotationSetpoint = 120;
//         break;
//       case 6:
//         rotationSetpoint = 120;
//         break;
//       case 7:
//         rotationSetpoint = 180;
//         break;
//       case 8:
//         rotationSetpoint = -120;
//         break;
//       case 9:
//         rotationSetpoint = -60;
//         break;
//       case 10:
//         rotationSetpoint = 0;
//         break;
//       case 11:
//         rotationSetpoint = 60;
//         break;
//       default:
//         rotationSetpoint = 1;
//         break;
//     }

//     if (rotationSetpoint != 1){
//       speedTurn = PIDControllerTurn.calculate(swerveSubs.getRotation2d().getDegrees(), rotationSetpoint);
//       speedX = PIDControllerX.calculate(LimelightHelpers.getTX("limelight"), 0);
//     } else {
//       speedTurn = zSpeed;
//     }
    

//     // square the speed values to make for smoother acceleration

//     /* * * SETTING SWERVE STATES * * */
//     swerveSubs.drive(forwardSpeed, speedX, speedTurn, false, 0.4);
//     System.out.println(tagID);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     swerveSubs.stopModules();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }

//   /* * * ADDED METHODS * * */
//   public double deadzone(double num) {
//     return Math.abs(num) > 0.03 ? num : 0; // CHECKME test optimal offset
//   }

//   private static double modifyAxis(double num) {
//     // Square the axis
//     num = Math.copySign(num * num, num);

//     return num;
//   }

//   public void periodic() {

//   }
// }


package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

@Logged
public class LimelightLineup extends Command {
  private SwerveSubsystem swerveSubs;

  private DoubleSupplier xSupplier, ySupplier, zSupplier;
  private BooleanSupplier fieldOriented;
  private double speedX, speedTurn;
  private PIDController PIDControllerX;
  private PIDController PIDControllerTurn;
  private double rotationSetpoint;


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
  public LimelightLineup(SwerveSubsystem swerveSubs, DoubleSupplier xSupplier) {
    this.swerveSubs = swerveSubs;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.fieldOriented = fieldOriented;
    PIDControllerTurn = new PIDController(0.03, 0, 0);
    PIDControllerX = new PIDController(0.01, 0.000, 0.0000);
    PIDControllerTurn.enableContinuousInput(-180, 180);
    PIDControllerX.setTolerance(0.25);
    addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /* * * ALTERING VALUES * * */
    // Joystick values -> double
    double forwardSpeed = xSupplier.getAsDouble();
    //double zSpeed = zSupplier.getAsDouble();

    int tagID = (int) LimelightHelpers.getFiducialID("limelight");

    switch (tagID) {
      case 17:
        rotationSetpoint = 60;
        break;
      case 18:
        rotationSetpoint = 0;
        break;
      case 19:
        rotationSetpoint = -60;
        break;
      case 20:
        rotationSetpoint = -120;
        break;
      case 21:
        rotationSetpoint = 180;
        break;
      case 22:
        rotationSetpoint = 120;
        break;
      case 6:
        rotationSetpoint = 120;
        break;
      case 7:
        rotationSetpoint = 180;
        break;
      case 8:
        rotationSetpoint = -120;
        break;
      case 9:
        rotationSetpoint = -60;
        break;
      case 10:
        rotationSetpoint = 0;
        break;
      case 11:
        rotationSetpoint = 60;
        break;
      default:
        rotationSetpoint = rotationSetpoint;
        break;
    }

    if (LimelightHelpers.getTargetCount("limelight") != 0){
      speedX = PIDControllerX.calculate(LimelightHelpers.getTX("limelight"), 3.5);
    }else{
      speedX = 0;
    }
    speedTurn = PIDControllerTurn.calculate(swerveSubs.getRotation2d().getDegrees(), rotationSetpoint);

    // square the speed values to make for smoother acceleration

    /* * * SETTING SWERVE STATES * * */
    swerveSubs.drive(forwardSpeed * 0.5, speedX, speedTurn, false, 0.4);
    System.out.println(rotationSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubs.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
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
