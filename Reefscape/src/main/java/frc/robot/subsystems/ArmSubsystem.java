// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private static SparkFlex armMotor;   
  private static RelativeEncoder armEncoder;
  private static SparkClosedLoopController armClosedLoopController;

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    armMotor = new SparkFlex(ArmConstants.armMotorId, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();


    armMotor.configure(Configs.ArmConfig.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armClosedLoopController = armMotor.getClosedLoopController();
  }

  public static void armUp(){
    if (armEncoder.getPosition() < 500){
      armMotor.set(1);
    }else{
      armMotor.set(0);
    }
  }

  public static void armDown(){
      armMotor.set(-1);
  }

  public static void armToPosition(double position){
    armClosedLoopController.setReference(position, ControlType.kPosition);
  }

  public static void armStop(){
    armMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ArmEncoder", armEncoder.getPosition());
    // This method will be called once per scheduler run
  }

}
