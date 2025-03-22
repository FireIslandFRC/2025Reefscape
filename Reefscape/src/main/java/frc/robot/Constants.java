// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

  public static final int PhID = 15;

  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class TargetLocationConstants{
    public static Pose2d slicePose1 = new Pose2d(6.5, 4, Rotation2d.fromDegrees(180));
    public static Pose2d slicePose2 = new Pose2d(5.5, 2.5, Rotation2d.fromDegrees(120));
    public static Pose2d slicePose3 = new Pose2d(3.5, 2.5, Rotation2d.fromDegrees(60));
    public static Pose2d slicePose4 = new Pose2d(2.6, 4, Rotation2d.fromDegrees(0));
    public static Pose2d slicePose5 = new Pose2d(3.5, 5.5, Rotation2d.fromDegrees(-60));
    public static Pose2d slicePose6 = new Pose2d(5.5, 5.5, Rotation2d.fromDegrees(-120));

    public static Pose2d coralLoad1 = new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(120));
    public static Pose2d coralLoad2 = new Pose2d(1.5, 1.5, Rotation2d.fromDegrees(-120));

    public static Pose2d cage1 = new Pose2d(7.5, 7.3, Rotation2d.fromDegrees(90));
    public static Pose2d cage2 = new Pose2d(7.5, 6.1, Rotation2d.fromDegrees(90));
    public static Pose2d cage3 = new Pose2d(7.5, 5, Rotation2d.fromDegrees(90));

    public static Pose2d currentTarget = TargetLocationConstants.slicePose1;

  }

  public static class SwerveConstants {
    //SDS L2 
    public static final boolean ROTATION_ENCODER_DIRECTION = false; //CHECKME not sure how works?

    public static final int PIGEON_ID = 30;

    /* * * MEASUREMENTS * * */
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double TRACK_WIDTH = Units.inchesToMeters(25);
    public static final double WHEEL_BASE = Units.inchesToMeters(25); //FIXME update
    
    public static final double DRIVE_GEAR_RATIO = 6.75 / 1;
    public static final double ROTATION_GEAR_RATIO = 150 / 7;
    
    public static final double VOLTAGE = 7.2;

    /* * * SWERVE DRIVE KINEMATICS * * */
    // ORDER IS ALWAYS FL, BL, FR, BR 
    //pos x is out in front, pos y is to the left CHECKME
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      
    //  // front left
    //   new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
    //   // back left
    //   new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
    //   // front right
    //   new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
    //   // back right
    //   new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)

      // front right
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      // back left
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
      // front left
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      // back right
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

    /* * * FRONT LEFT * * */
    public static class FrontLeft {
      public static final int DRIVE_PORT = 1;
      public static final int ROTATION_PORT = 2;
      public static final int ABSOLUTE_ENCODER_PORT = 21;
      //public static final double OFFSET = 144.6; // NOTE CompFrame
      public static final double OFFSET = -125; 
      public static final boolean DRIVE_INVERTED = false;
      public static final boolean ROTATION_INVERTED = true;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED);
    }

    /* * * FRONT RIGHT * * */
    public static class FrontRight {
      public static final int DRIVE_PORT = 3;
      public static final int ROTATION_PORT = 4;
      public static final int ABSOLUTE_ENCODER_PORT = 22;
      //public static final double OFFSET = 0 -17; // NOTE CompFrame
      public static final double OFFSET = 144;
      public static final boolean DRIVE_INVERTED = false; 
      public static final boolean ROTATION_INVERTED = true; 

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED);
    }

    /* * * BACK LEFT * * */
    public static class BackLeft {
      public static final int DRIVE_PORT = 5;
      public static final int ROTATION_PORT = 6;
      public static final int ABSOLUTE_ENCODER_PORT = 23;
      //public static final double OFFSET = -161; // NOTE CompFrame
      public static final double OFFSET = 51;
      public static final boolean DRIVE_INVERTED = false;
      public static final boolean ROTATION_INVERTED = true; 

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED);
    }

    /* * * BACK RIGHT * * */
    public static class BackRight {
      public static final int DRIVE_PORT = 7;
      public static final int ROTATION_PORT = 8;
      public static final int ABSOLUTE_ENCODER_PORT = 24;
      //public static final double OFFSET = 52.5; // NOTE CompFrame
      public static final double OFFSET = -85;
      public static final boolean DRIVE_INVERTED = false; 
      public static final boolean ROTATION_INVERTED = true; 

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED);
    }

    /* * * CONVERSIONS FOR ENCODERS * * */
    //velocity in meters per sec instead of RPM 
    public static final double DRIVE_ENCODER_POSITION_CONVERSION = ((2 * Math.PI * (WHEEL_DIAMETER/2))) / DRIVE_GEAR_RATIO; //drive enc rotation
    //velocity in meters instead of rotations 
    public static final double DRIVE_ENCODER_VELOCITY_CONVERSION = DRIVE_ENCODER_POSITION_CONVERSION / 60; //drive enc speed 

    /* * * PID VALUES FOR TURNING MOTOR PID * * */
    public static final double KP_TURNING = 0.01;
    public static final double KI_TURNING = 0.00;
    public static final double KD_TURNING = 0.00;

    /* * * MAX * * */
    public static final double MAX_SPEED = 3; //12.0 ft/s CHECKME
    public static final double MAX_ROTATION = MAX_SPEED / Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);

  }

  /* * * Hand * * */
  public static class HandConstants {
    public static final int wristMotorId = 11;
    public static final int handMotorId = 12;
  }

  /* * * Climber * * */
  public static class ClimberConstants {
    public static final int climbMotorId = 10;
    public static final int servoId = 9;
  }
  
  /* * * Arm * * */
  public static class ArmConstants {
    public static final int armMotorId = 9;
  }
  
  /* * * Processor * * */
  public static class ProcessorConstants {
    public static final int processorPivot = 13;
    public static final int processorWheel = 14;
  }

  public final class Vision {
    public static final String OBJ_DETECTION_LIMELIGHT_NAME = "limelight-neural";

    public static final String LIMELIGHT_SHUFFLEBOARD_TAB = "Vision";

    public static final double ALLOWABLE_POSE_DIFFERENCE = 0.5;
    public static final double MAX_TAG_DISTANCE = 3.5;

    public static final Translation2d FIELD_CORNER = new Translation2d(17.54, 8.02);
    public static final Translation2d FIELD_CORNER_FOR_INTAKE = new Translation2d(16.65, 7.5);


    // how many degrees back is your limelight rotated from perfectly vertical?
    public static final double limelightMountAngleDegrees = 22.0;
    // distance from the center of the Limelight lens to the floor
    public static final double limelightLensHeightInches = 0.233;
    // height of april tags from the floor in meters
    public static final double AprilTagHeight = 1.335;
  }

}