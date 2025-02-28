package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.HandConstants;

public class HandSubsystem extends SubsystemBase {
    
    private static SparkMax wristMotor;
    private static SparkMax handMotor;
    private static SparkAbsoluteEncoder wristAbsEncoder;
    private static SparkClosedLoopController wristClosedLoopController;
    
    public HandSubsystem(){
        wristMotor = new SparkMax(HandConstants.wristMotorId, MotorType.kBrushless);

        wristMotor.configure(Configs.EEConfig.wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        handMotor = new SparkMax(HandConstants.handMotorId, MotorType.kBrushless);

        handMotor.configure(Configs.EEConfig.handConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        wristClosedLoopController = wristMotor.getClosedLoopController();

        wristAbsEncoder = wristMotor.getAbsoluteEncoder();
    }

    public static void PivotUp(){
        //if (wristAbsEncoder.getPosition() < 190){
            wristMotor.set(0.1);
        //} else {
        //    wristMotor.set(0);
        //}
    }

    public static void PivotDown(){
        //if (wristAbsEncoder.getPosition() > 70){
            wristMotor.set(-0.2);
        //}else{
        //    wristMotor.set(0);
        //}
    }

    public static void WristToAngle(double angle){
        wristClosedLoopController.setReference(angle, ControlType.kPosition);
    }

    public static void PivotStop(){
        wristMotor.set(0);
    }

    public static void CoralIn(){
        handMotor.set(0.50);
    }

    public static void CoralOut(){
        handMotor.set(-0.75);
    }

    public static void CoralStop(){
        handMotor.set(0);
    }

    @Override
    public void periodic(){
    SmartDashboard.putNumber("Wrist", wristAbsEncoder.getPosition());

    }

}