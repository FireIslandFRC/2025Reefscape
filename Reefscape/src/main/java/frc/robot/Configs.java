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
                                        .zeroOffset(0.19444);
                        wristConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        .pid(.003,0.000,0); //P:0.008

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

        public static final class ProcessorConfig {

                public static final SparkMaxConfig processorPivotConfig = new SparkMaxConfig(); //CHECKME untested SparkMax to Vortex
                public static final SparkMaxConfig processorWheelConfig = new SparkMaxConfig(); //CHECKME untested SparkMax to Vortex

                static {

                        processorPivotConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(80);
                        processorPivotConfig.encoder
                                        .positionConversionFactor(1); //WATCHME possible change to inches
                        processorPivotConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .pid(.08,0,0); //P:0.008

                        processorWheelConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(80);
                
                }
        }
}
