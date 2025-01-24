// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }

  public static class ArmConstants {
    public static final int armMotorId = 9;
  }

  public static class SwerveConstants {
    //SDS L2 
    public static final boolean ROTATION_ENCODER_DIRECTION = false; 

    /* * * MEASUREMENTS * * */
    //FIXME REPLACE WITH VALUES OF ACTUAL BASE 
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double TRACK_WIDTH = Units.inchesToMeters(28);
    public static final double WHEEL_BASE = Units.inchesToMeters(28);
    
    public static final double DRIVE_GEAR_RATIO = 6.75 / 1;
    public static final double ROTATION_GEAR_RATIO = 150 / 7;
    
    public static final double VOLTAGE = 7.2;

    /* * * SWERVE DRIVE KINEMATICS * * */
    // ORDER IS ALWAYS FL, BL, FR, BR 
    //pos x is out in front, pos y is to the left 
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      
      // front left
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      // back left
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
      // front right
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      // back right
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
      
      //KILLME 1/18/25
      /* //front left 
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), 

      //back left 
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),

      //front right 
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), 

      //back right 
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2) */

    );

    /* * * FRONT LEFT * * */
    //FIXME FILL IN VALUES FOR FRONT LEFT 
    public static class FrontLeft {
      public static final int DRIVE_PORT = 1;
      public static final int ROTATION_PORT = 2;
      public static final int ABSOLUTE_ENCODER_PORT = 21;
      public static final double OFFSET = -32.8;
      public static final boolean DRIVE_INVERTED = true;
      public static final boolean ROTATION_INVERTED = true;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED);
    }

    /* * * FRONT RIGHT * * */
    //FIXME FILL IN VALUES FOR FRONT RIGHT 
    public static class FrontRight {
      public static final int DRIVE_PORT = 3;
      public static final int ROTATION_PORT = 4;
      public static final int ABSOLUTE_ENCODER_PORT = 22;
      public static final double OFFSET = -5.5;
      public static final boolean DRIVE_INVERTED = true; 
      public static final boolean ROTATION_INVERTED = true; 

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED);
    }

    /* * * BACK LEFT * * */
    //FIXME FILL IN VALUES FOR BACK LEFT 
    public static class BackLeft {
      public static final int DRIVE_PORT = 5;
      public static final int ROTATION_PORT = 6;
      public static final int ABSOLUTE_ENCODER_PORT = 23;
      public static final double OFFSET = -170;
      public static final boolean DRIVE_INVERTED = false; 
      public static final boolean ROTATION_INVERTED = true; 

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED);
    }

    /* * * BACK RIGHT * * */
    //FIXME FILL IN VALUES FOR BACK RIGHT 
    public static class BackRight {
      public static final int DRIVE_PORT = 7;
      public static final int ROTATION_PORT = 8;
      public static final int ABSOLUTE_ENCODER_PORT = 24;
      public static final double OFFSET = -114.5;
      public static final boolean DRIVE_INVERTED = true; 
      public static final boolean ROTATION_INVERTED = true; 

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED);
    }

    /* * * CONVERSIONS FOR ENCODERS * * */
    //velocity in meters per sec instead of RPM 
    public static final double DRIVE_ENCODER_POSITION_CONVERSION = ((2 * Math.PI * (WHEEL_DIAMETER/2))) / DRIVE_GEAR_RATIO; //drive enc rotation
    //velocity in meters instead of rotations 
    public static final double DRIVE_ENCODER_VELOCITY_CONVERSION = DRIVE_ENCODER_POSITION_CONVERSION / 60; //drive enc speed 

    /* * * PID VALUES FOR TURNING MOTOR PID * * */

    
    public static final double KP_TURNING = 0.02;
    public static final double KI_TURNING = 0;
    public static final double KD_TURNING = 0.00;

    /* * * MAX * * */
    public static final double MAX_SPEED = 3; //12.0 ft/s 
    public static final double MAX_ROTATION = MAX_SPEED / Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
  
  }
}