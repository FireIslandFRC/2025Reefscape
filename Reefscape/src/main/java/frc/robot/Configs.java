package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.SwerveConstants;

public final class Configs {
        public static final class MAXSwerveModule {
                public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
                public static final SparkFlexConfig turningConfig = new SparkFlexConfig();

                static {

                        drivingConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(25);
                        drivingConfig.encoder
                                        .positionConversionFactor(SwerveConstants.DRIVE_ENCODER_POSITION_CONVERSION) // meters
                                        .velocityConversionFactor(SwerveConstants.DRIVE_ENCODER_VELOCITY_CONVERSION); // meters per second
                        drivingConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        // These are example gains you may need to them for your own robot!
                                        //.pid(0.04, 0, 0)
                                        .velocityFF(0.223)
                                        .outputRange(-1, 1);

                        turningConfig
                                        .idleMode(IdleMode.kCoast)
                                        .smartCurrentLimit(20);
                        turningConfig.encoder
                                        // Invert the turning encoder, since the output shaft rotates in the opposite
                                        // direction of the steering motor in the MAXSwerve Module.
                                        .positionConversionFactor(.0465)//.085, 0425, 042
                                        .velocityConversionFactor(1); // radians per second
                        turningConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                                        .pid(2, 0, 0)
                                        .velocityFF(.223)
                                        .outputRange(-1, 1);
                                        // Enable PID wrap around for the turning motor. This will allow the PID
                                        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                                        // to 10 degrees will go through 0 rather than the other direction which is a
                                        // longer route.
                                        //.positionWrappingEnabled(true)
                                        //.positionWrappingInputRange(0, (2 * Math.PI));
                }
        }
}