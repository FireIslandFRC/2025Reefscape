// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private SparkFlex armMotor;

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    armMotor = new SparkFlex(ArmConstants.armMotorId, MotorType.kBrushless);

    armMotor.configure(Configs.MAXSwerveModule.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void ArmUp(){
    armMotor.set(0.5);
  }

  public void ArmDown(){
    armMotor.set(-0.5);
  }

  public void armStop(){
    armMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
