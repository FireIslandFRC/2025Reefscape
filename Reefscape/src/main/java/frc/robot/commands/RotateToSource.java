package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateToSource extends Command {
  private SwerveSubsystem swerveSubs; 

  private double angle, rotationSpeed;
  private PIDController rotationController;

  private DoubleSupplier xSupplier, ySupplier;
  /* * * CONSTRUCTOR * * */
  /* 
   * @param swerveSubs the swerve subsystem 
   * @param xSupplier value input for strafe on x-axis 
   * @param ySupplier value input for strafe on y-axis 
   * @param zSupplier value input for rotation 
   * @param fieldOriented whether or not we want the bot to run in field oriented 
   */
  public RotateToSource(SwerveSubsystem swerveSubs, DoubleSupplier xSupplier, DoubleSupplier ySupplier, double angle) {
    this.swerveSubs = swerveSubs; 
    this.angle = angle;

    this.xSupplier = xSupplier; 
    this.ySupplier = ySupplier;
    addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.setPID(0.2,0,0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = xSupplier.getAsDouble(); 
    double ySpeed = ySupplier.getAsDouble();

    // SmartDashboard.putNumber("x speed", xSpeed);
    // SmartDashboard.putNumber("y speed", ySpeed);
    // SmartDashboard.putNumber("z speed", zSpeed);


    //apply deadzone to speed values 
    xSpeed = deadzone(xSpeed); 
    ySpeed = deadzone(ySpeed); 

    rotationSpeed = rotationController.calculate(swerveSubs.getRotation2d().getDegrees(), angle);
    //square the speed values to make for smoother acceleration 

    /* * * SETTING SWERVE STATES * * */

    swerveSubs.drive(xSpeed, ySpeed, rotationSpeed, true, 0.75);
    
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
public double deadzone(double num){
    return Math.abs(num) > 0.1 ? num : 0;
}

private static double modifyAxis(double num) {
  // Square the axis
  num = Math.copySign(num * num, num);

  return num;
}


public void periodic() {

}
}
