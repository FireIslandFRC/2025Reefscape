package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.HandConstants;

public class HandSubsystem extends SubsystemBase {
    
    private static SparkMax wristMotor;
    private static SparkMax handMotor;

    
    public HandSubsystem(){
        wristMotor = new SparkMax(HandConstants.wristMotorId, MotorType.kBrushless);

        wristMotor.configure(Configs.EEConfig.wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        handMotor = new SparkMax(HandConstants.handMotorId, MotorType.kBrushless);

        handMotor.configure(Configs.EEConfig.handConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static void PivotUp(){
        wristMotor.set(0.2);
    }

    public static void PivotDown(){
        wristMotor.set(-0.2);
    }

    public static void PivotStop(){
        wristMotor.set(0);
    }

    public static void CoralIn(){
        handMotor.set(0.5);
    }

    public static void CoralOut(){
        handMotor.set(-0.5);
    }

    public static void CoralStop(){
        handMotor.set(0);
    }

    @Override
    public void periodic(){

    }

}