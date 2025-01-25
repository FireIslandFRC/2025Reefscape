package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class HandSubsystem extends SubsystemBase {
    
    private TalonFX climbMotor;

    private TalonFXConfiguration ClimbingConfig;
    
    public HandSubsystem(){
        climbMotor = new TalonFX(ClimberConstants.climbMotorId);

        //climbMotor.configure(Configs.MAXSwerveModule.climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); NOTE: Rev Config
        ClimbingConfig = new TalonFXConfiguration();

        ClimbingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        ClimbingConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        ClimbingConfig.CurrentLimits.SupplyCurrentLimit = 20;
        ClimbingConfig.Feedback.SensorToMechanismRatio = 1;

        climbMotor.getConfigurator().apply(ClimbingConfig);
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
