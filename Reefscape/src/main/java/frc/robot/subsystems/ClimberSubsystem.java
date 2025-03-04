package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    
    private TalonFX climbMotor;
    private TalonFXConfiguration climbMotorConfig;
    private Servo ratchetServo;
    
    public ClimberSubsystem(){
        climbMotor = new TalonFX(ClimberConstants.climbMotorId);
        ratchetServo = new Servo(ClimberConstants.servoId);

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
    
    public void climbStop(){
        climbMotor.set(0);
    }

    @Override
    public void periodic(){

    }

}
