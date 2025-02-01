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
                                        .smartCurrentLimit(25);
                        drivingConfig.encoder
                                        .positionConversionFactor(.29) // meters
                                        .velocityConversionFactor(1); // meters per second
                        drivingConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        // These are example gains you may need to them for your own robot!
                                        .pid(0.4, 0, 0)
                                        .velocityFF(0.223)
                                        .outputRange(-1, 1);
                }
        }
}
