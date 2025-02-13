package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.SwerveConstants;

public final class Configs {
        public static final class MAXSwerveModule {
                public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
                public static final SparkFlexConfig turningConfig = new SparkFlexConfig();

                static {

                        drivingConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(25)
                                        .inverted(true);
                        drivingConfig.encoder
                                        .positionConversionFactor(SwerveConstants.DRIVE_ENCODER_POSITION_CONVERSION) // meters   CHECKME make sure right conversion
                                        .velocityConversionFactor(SwerveConstants.DRIVE_ENCODER_VELOCITY_CONVERSION); // meters per second

                        turningConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(20)
                                        .inverted(true);
                }
        }
}
