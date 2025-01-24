package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    
    private SparkFlex climbMotor;

    
    public ClimberSubsystem(){
        climbMotor = new SparkFlex(ClimberConstants.climbMotorId, MotorType.kBrushless);

        climbMotor.configure(Configs.MAXSwerveModule.climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void climbUp(){
        climbMotor.set(0.5);
    }

    public void climbDown(){
        climbMotor.set(-0.5);
    }

    @Override
    public void periodic(){

    }


    public void ClimbStop(){
        climbMotor.set(0);
    }





}
