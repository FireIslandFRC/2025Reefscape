package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.SwerveConstants;

public final class Configs {
        public static final class SwerveModuleConfig {
                public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
                public static final SparkFlexConfig turningConfig = new SparkFlexConfig();

                static {

                        drivingConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(40)
                                        .inverted(true);
                        drivingConfig.encoder
                                        .positionConversionFactor(SwerveConstants.DRIVE_ENCODER_POSITION_CONVERSION) // meters   CHECKME make sure right conversion
                                        .velocityConversionFactor(SwerveConstants.DRIVE_ENCODER_VELOCITY_CONVERSION); // meters per second

                        turningConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(20)
                                        .inverted(true); //NOTE: DONT FORGET
                }
        }

        public static final class EEConfig {
                
                public static final SparkMaxConfig wristConfig = new SparkMaxConfig();
                public static final SparkMaxConfig handConfig = new SparkMaxConfig();

                static {

                        wristConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(40);
                        wristConfig.absoluteEncoder
                                        .positionConversionFactor(360)
                                        .zeroOffset(0.0340288);
                        wristConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        .pid(.008,0,0);

                        handConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(40);
                        handConfig.encoder
                                        .positionConversionFactor(1);
                }
        }

        public static final class ArmConfig {

                public static final SparkFlexConfig armConfig = new SparkFlexConfig(); //CHECKME untested SparkMax to Vortex

                static {

                        armConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(80);
                        armConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .pid(.015,0,0);
                        armConfig.encoder
                                        .positionConversionFactor(1); //WATCHME possible change to inches
                
                }
        }
}
