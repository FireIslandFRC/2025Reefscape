package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    
    private TalonFX climbMotor;
    private TalonFXConfiguration climbMotorConfig;
    private Servo ratchetServo;

    
    public ClimberSubsystem(){
        climbMotor = new TalonFX(ClimberConstants.climbMotorId);
        ratchetServo = new Servo(9);

        climbMotorConfig = new TalonFXConfiguration();

        climbMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climbMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climbMotor.getConfigurator().apply(climbMotorConfig);
    }

    public void climbUp(){
        climbMotor.set(0.5);
    }

    public void climbDown(){
        climbMotor.set(-0.5);
    }

    public void openRatchet(){
        ratchetServo.setAngle(30);
    }

    public void closedRatchet(){
        ratchetServo.setAngle(70);
    }

    @Override
    public void periodic(){

    }


    public void ClimbStop(){
        climbMotor.set(0);
    }





}
