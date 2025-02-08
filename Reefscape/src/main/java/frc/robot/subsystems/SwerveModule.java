package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveModuleConstants;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

//import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import java.util.Map;

import com.ctre.phoenix6.StatusSignal.SignalMeasurement;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {
    /* * * INITIALIZATION * * */

    public int moduleID;
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


        driveMotor.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        rotationMotor.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);




        /* * * DRIVE MOTOR * * */
        // CONFIGURATIONS
        // driveMotor.setInverted(moduleConstants.driveInverted);
        // driveMotor.setIdleMode();
        // driveMotor.setSmartCurrentLimit(25);

        // set conversion factor for drive enc
        /*
         * driveEncoder.setVelocityConversionFactor(SwerveConstants.
         * DRIVE_ENCODER_VELOCITY_CONVERSION); // reads velocity
         * // in meters per
         * // second instead
         * // of RPM
         * driveEncoder.setPositionConversionFactor(SwerveConstants.
         * DRIVE_ENCODER_POSITION_CONVERSION); // reads velocity
         * // in meters
         * // instead of
         * // rotations
         */

        /* * * ROTATION MOTOR * * */
        // CONFIGURATIONS
        /*
         * rotationMotor.setInverted(moduleConstants.rotationInverted);
         * rotationMotor.setIdleMode(IdleMode.kBrake);
         * rotationMotor.setSmartCurrentLimit(25); // set current limit to 25 amps to
         * prevent browning out in the middle of
         * // driving
         */

        // configure rotation absolute encoder
        absoluteEncoder.getConfigurator().apply(
                new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(0.5)); // abs
        // abs
        // enc
        // is
        // now
        // +-180
        /* absoluteEncoder.getConfigurator().apply(new
         MagnetSensorConfigs().withMagnetOffset(moduleConstants.angleOffset/360));*/
        // //implements encoder offset
        absoluteEncoder.getConfigurator()
                .apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)); // positive
                                                                                                                       // rotation
                                                                                                                       // occurs
                                                                                                                       // when
                                                                                                                       // magnet
                                                                                                                       // is
                                                                                                                       // spun
                                                                                                                       // counter-clockwise
                                                                                                                       // when
                                                                                                                       // observer
                                                                                                                       // is
                                                                                                                       // facing
                                                                                                                       // the
                                                                                                                       // LED
                                                                                                                       // side
                                                                                                                       // of
                                                                                                                       // CANCoder

        // configure rotation PID controller
        rotationPID = new PIDController(
                SwerveConstants.KP_TURNING,
                SwerveConstants.KI_TURNING,
                SwerveConstants.KD_TURNING);

                //rotationPID.setTolerance(1);
                //rotationPID.setIZone(100);
                
        rotationPID.enableContinuousInput(-180, 180); // Continuous input considers min & max to be the same point;
                                                      // calculates the shortest route to the setpoint

                                      //SmartDashboard.getNumber("PID", rotationPID.getSetpoint());
        //rotationPID.

        SmartDashboard.putNumber("Offset" + absoluteEncoder.getDeviceID(), moduleConstants.angleOffset);
                   
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
        //SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

        //double rotationOutput = rotationPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees()); //NOTE removed because optimized broken fix?
        double rotationOutput = rotationPID.calculate(getState().angle.getDegrees(), desiredState.angle.getDegrees());
        //double rotationOutput = rotationPID.calculate(absoluteEncoder.getAbsolutePosition().getValueAsDouble(), desiredState.angle.getDegrees());

        //double fixedRotationOutput = (((rotationOutput - -400) * (1 - -1)) / (400 - -400)) + -1;

        //rotationMotor.set(MathUtil.clamp(rotationOutput, -1, 1));
        rotationMotor.set(rotationOutput);


        //SmartDashboard.putNumber("FixedRotationOutput" + absoluteEncoder.getDeviceID() , MathUtil.clamp(rotationOutput, -.1, .1));
        SmartDashboard.putNumber("RotationSpeed" + absoluteEncoder.getDeviceID() , rotationOutput);
        //SmartDashboard.putNumber("getState().angle.getDegrees()" + absoluteEncoder.getDeviceID() , getState().angle.getDegrees());
        //SmartDashboard.putNumber("desiredState.angle.getDegrees()" + absoluteEncoder.getDeviceID() , desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Error" + absoluteEncoder.getDeviceID(), rotationPID.getError());
        SmartDashboard.putNumber("SetPoint" + absoluteEncoder.getDeviceID(), rotationPID.getSetpoint());

        //SmartDashboard.putNumber("P" + absoluteEncoder.getDeviceID() , rotationPID.get);
        //driveMotor.set(optimizedState.speedMetersPerSecond / SwerveConstants.MAX_SPEED * SwerveConstants.VOLTAGE); //NOTE removed because optimized broken fix?
        driveMotor.set(desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED * SwerveConstants.VOLTAGE);

        //SmartDashboard.putString("S[" + absoluteEncoder.getDeviceID() + "] DESIRED ANG DEg 2",
        //desiredState.toString());

        /*SmartDashboard.putString("S[" + absoluteEncoder.getDeviceID() + "] OPTIMIZED STATE",
        optimizedState.toString());*/ //NOTE optimized printing.

        //SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] Rotation output 2", rotationOutput);
    }

    public void setAngle(SwerveModuleState desiredState) {
        System.out.println("set angle run");
        //SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle); NOTE optimizer buggy

        //double rotationOutput = rotationPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());  // NOTE replaced optimized with desired
        double rotationOutput = rotationPID.calculate(getState().angle.getDegrees(), desiredState.angle.getDegrees());
        rotationMotor.set(rotationOutput);
        driveMotor.set(0);

        //SmartDashboard.putString("S[" + absoluteEncoder.getDeviceID() + "] DESIRED ANG DEG",
        //desiredState.toString());

        /*SmartDashboard.putString("S[" + absoluteEncoder.getDeviceID() + "] OPTIMIZED STATE",
        optimizedState.toString());*/ //NOTE optimized printing.
        
        //SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] Rotation output", rotationOutput);
    }

    public void stop() {
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    public void print() {
        SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] ABS ENC DEG", getAbsoluteEncoderDegrees());
        
        // SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] DRIVE SPEED", getDriveVelocity());
        // SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] ROTATION SPEED",
        //         absoluteEncoder.getVelocity().getValueAsDouble());
        // SmartDashboard.putString("S[" + absoluteEncoder.getDeviceID() + "] CURRENT STATE", getState().toString());

    }
}