// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private static SparkMax armMotor;
  private static RelativeEncoder armEncoder;
  private static double SETPOS1, SETPOS2, SETPOS3;

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    armMotor = new SparkMax(ArmConstants.armMotorId, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();


    armMotor.configure(Configs.ArmConfig.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public static void armUp(){
    armMotor.set(1);
  }

  public static void armDown(){
    armMotor.set(-1);
  }

  public static void armToPosition(int position){
    //FIXME implement encoder
  }

  public static void armStop(){
    armMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
