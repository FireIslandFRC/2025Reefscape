package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ProcessorConstants;

public class ProcessorSubsystem extends SubsystemBase {
    
    private SparkMax processorPivot;
    private SparkMax processorWheel;
    private static RelativeEncoder processorEncoder;
    
    public ProcessorSubsystem(){
        processorPivot = new SparkMax(ProcessorConstants.processorPivot, MotorType.kBrushless);
        processorWheel = new SparkMax(ProcessorConstants.processorWheel, MotorType.kBrushless);

        processorPivot.configure(Configs.ProcessorConfig.processorPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        processorWheel.configure(Configs.ProcessorConfig.processorWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        processorEncoder = processorPivot.getEncoder();
    }

    public void processorPivotDown(){
        processorPivot.set(-0.5);
    }

    public void processorPivotUp(){
        processorPivot.set(0.5);
    }

    public void processorWheelOut(){
        processorPivot.set(-0.5);
    }

    public void processorWheelIn(){
        processorPivot.set(0.5);
    }

    public void processorPivotStop(){
        processorPivot.set(0);
    }

    public void processorWheelStop(){
        processorWheel.set(0);
    }

    @Override
    public void periodic(){

    }

}