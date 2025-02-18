package frc.robot.subsystems;

import frc.robot.SwerveModuleConstants;
import frc.robot.Configs;
import frc.robot.Constants.SwerveConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {
    /* * * INITIALIZATION * * */

    protected int moduleID;

    // initialize motors
    private SparkFlex driveMotor;
    private SparkFlex rotationMotor;

    // initialize encoders
    private CANcoder absoluteEncoder;
    private RelativeEncoder driveEncoder;

    // init PID Controller for turning
    private PIDController rotationPID;

    // init info
    private double encOffset;

    /* * * CONSTRUCTOR * * */
    /*
     * @param moduleID the id of the module
     * 
     * @param moduleConstants a SwerveModuleConstants obj
     */

    public SwerveModule(int moduleID, SwerveModuleConstants moduleConstants) {
        this.moduleID = moduleID; // used to differentiate between the four swerve modules in the SwerveSubsystem
                                  // class
        encOffset = moduleConstants.angleOffset;


        // instantiate drive motor and encoder
        driveMotor = new SparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();

        // instantiate rotation motor and absolute encoder
        rotationMotor = new SparkFlex(moduleConstants.rotationMotorID, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(moduleConstants.cancoderID);

        // configures drive and rotation motors
        driveMotor.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        rotationMotor.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        /* * * ABSOLUTE ENCODER CONFIG * * */
        // abs enc is now +-180
        absoluteEncoder.getConfigurator().apply(
                new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(0.5));
   
        // positive rotation occurs when magnet is spun counter-clockwise when observer is facing the LED side of CANCoder        
        absoluteEncoder.getConfigurator()
                .apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

        /* * * ROTATION PID CONFIGURE * * */
        rotationPID = new PIDController(
                SwerveConstants.KP_TURNING,
                SwerveConstants.KI_TURNING,
                SwerveConstants.KD_TURNING);

        // Continuous input considers min & max to be the same point;
        rotationPID.enableContinuousInput(-180, 180);

    }

    /* * * GET METHODS * * */
    private double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    private double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    private double getAbsoluteEncoderDegrees() {
        return ((absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360) - encOffset);
    }

    // returns a new SwerveModuleState representing the current drive velocity and
    // rotation motor angle
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAbsoluteEncoderDegrees()));
    }

    // returns a new SwerveModulePosition representing the current drive position
    // and rotation motor angle
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getAbsoluteEncoderDegrees()));
    }

    /* * * SET METHODS * * */
    public void setState(SwerveModuleState desiredState) {
        // optimize state so the rotation motor doesnt have to spin as much
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

        double rotationOutput = rotationPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees()); 

        rotationMotor.set(rotationOutput);

        // NOTE: Uncomment for PID tuning
        // SmartDashboard.putNumber("RotationSpeed" + absoluteEncoder.getDeviceID() , rotationOutput);
        // SmartDashboard.putNumber("Error" + absoluteEncoder.getDeviceID(), rotationPID.getError());
        // SmartDashboard.putNumber("SetPoint" + absoluteEncoder.getDeviceID(), rotationPID.getSetpoint());

        driveMotor.set(optimizedState.speedMetersPerSecond / SwerveConstants.MAX_SPEED * SwerveConstants.VOLTAGE);

    }

    public void setAngle(SwerveModuleState desiredState) {
        // optimize state so the rotation motor doesnt have to spin as much
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

        double rotationOutput = rotationPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees()); 

        rotationMotor.set(rotationOutput);

        // NOTE: Uncomment for PID tuning
        // SmartDashboard.putNumber("RotationSpeed" + absoluteEncoder.getDeviceID() , rotationOutput);
        // SmartDashboard.putNumber("Error" + absoluteEncoder.getDeviceID(), rotationPID.getError());
        // SmartDashboard.putNumber("SetPoint" + absoluteEncoder.getDeviceID(), rotationPID.getSetpoint());

        driveMotor.set(0);
        
    }

    // stops rotation and rotation motors
    public void stop() {
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    public void print() {
        // NOTE prints abs encoder direction in degrees
        SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] ABS ENC DEG", getAbsoluteEncoderDegrees());
        
    }
}