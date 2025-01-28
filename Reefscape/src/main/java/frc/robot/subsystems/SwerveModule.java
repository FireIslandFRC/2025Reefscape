package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.SwerveConstants;
import frc.robot.SwerveModuleConstants;

public class SwerveModule {
    /* * * INITIALIZATION * * */

    public int moduleID;
    // initialize motors
    private SparkFlex driveMotor;
    private SparkFlex rotationMotor;

    // initialize encoders
    private CANcoder absoluteEncoder;
    private RelativeEncoder driveEncoder;

    private SparkClosedLoopController rotationControl;

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

        rotationControl = rotationMotor.getClosedLoopController();

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
        // absoluteEncoder.getConfigurator().apply(new
        // MagnetSensorConfigs().withMagnetOffset(moduleConstants.angleOffset));
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
        rotationPID.enableContinuousInput(-180, 180); // Continuous input considers min & max to be the same point;
                                                      // calculates the shortest route to the setpoint

                                      //SmartDashboard.getNumber("PID", rotationPID.getSetpoint());
                   
    }

    /* * * GET METHODS * * */
    private double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    private double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    private double getAbsoluteEncoderDegrees() {
        return (absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360) - encOffset;
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

        rotationControl.setReference(1, ControlType.kPosition);

        /*double rotationOutput = rotationPID.calculate(getState().angle.getDegrees(), desiredState.angle.getDegrees());

        rotationMotor.set(rotationOutput);*/
        SmartDashboard.putNumber("Error" + absoluteEncoder.getDeviceID() , rotationPID.getError());
       // SmartDashboard.putNumber("P" + absoluteEncoder.getDeviceID() , rotationPID.get);
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
        // SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] ABS ENC DEG", getAbsoluteEncoderDegrees());
        // SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] DRIVE SPEED", getDriveVelocity());
        // SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] ROTATION SPEED",
        //         absoluteEncoder.getVelocity().getValueAsDouble());
        // SmartDashboard.putString("S[" + absoluteEncoder.getDeviceID() + "] CURRENT STATE", getState().toString());
        // SmartDashboard.putNumber("PID", rotationPID.getSetpoint());

    }
}