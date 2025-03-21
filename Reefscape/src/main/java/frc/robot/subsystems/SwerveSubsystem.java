package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class SwerveSubsystem extends SubsystemBase {
  /* * * INITIALIZATION * * */

  //initialize SwerveModules 
  private SwerveModule[] swerveModules; 

  //instantiate pigeon 
  public Pigeon2 pigeon = new Pigeon2(SwerveConstants.PIGEON_ID);

  //field2d
  public Field2d m_field;

  //instantiate poseEstimator
  private SwerveDrivePoseEstimator m_poseEstimator;

  //private Limelight limelight;


  // swervesubsystem constructor
  public SwerveSubsystem() {
    
    pigeon.reset();

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, SwerveConstants.FrontLeft.constants), 
      new SwerveModule(1, SwerveConstants.BackLeft.constants), 
      new SwerveModule(2, SwerveConstants.FrontRight.constants), 
      new SwerveModule(3, SwerveConstants.BackRight.constants)
    };

    //field2d
    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    m_poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.DRIVE_KINEMATICS, getRotation2d(), getModulePositions(), new Pose2d(0,0,new Rotation2d()));

    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                               // TODO change to resetOdometry that is in the Max swerve template
          this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                                // ChassisSpeeds. Also optionally outputs individual
                                                                // module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                          // holonomic drive trains
              new PIDConstants(1, 0.0, 0), // Translation PID constants
              new PIDConstants(1.5, 0.0, 0.5) // Rotation PID constants    //FIXME tune auto
              //new PIDConstants(1, 0.0, 0.5), // Translation PID constants
              //new PIDConstants(1, 0.0, 0.1) // Rotation PID constants    //FIXME tune auto
          ),
          config, // The robot configuration //CHECKME untested alliance color flip
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

  }

  /* * * RESET METHODS * * */
  public void resetPigeon() {
   if (DriverStation.getAlliance().isPresent()
      && DriverStation.getAlliance().get() == Alliance.Red) {
      pigeon.setYaw(180);
    } else {
      pigeon.setYaw(0);
    }
    //pigeon.setYaw(0);
  }
  
  // public void resetOdometry() {
  //   m_poseEstimator.resetPosition(getRotation2d(), getModulePositions(), new Pose2d());
  // }

  public void resetOdometry(Pose2d pose) {
    // int flipped;
    // if (DriverStation.getAlliance().isPresent()
    //   && DriverStation.getAlliance().get() == Alliance.Red) {
    //   flipped = 180;
    // } else {
    //   flipped = 0;
    // } 
    //m_poseEstimator.resetPosition(new Rotation2d(getRotation2d().getDegrees() + flipped), getModulePositions(), pose);
    m_poseEstimator.resetPosition(new Rotation2d(getRotation2d().getDegrees()), getModulePositions(), pose);
  }

  /* * * GET METHODS * * */
  //returns the Rotation2d object, a 2d coordinate represented by a point on the unit circle (the rotation of the robot)
  public Rotation2d getRotation2d() {
    return pigeon.getRotation2d();
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  // returns a ChassisSpeeds in robot relative
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return new ChassisSpeeds(SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates()).vxMetersPerSecond, SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates()).vyMetersPerSecond, SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond);
  }

  //returns the states of the swerve modules in an array 
  //getState uses drive velocity and module rotation 
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4]; 

    for (SwerveModule swerveMod : swerveModules) {
      states[swerveMod.moduleID] = swerveMod.getState();
    }

    return states; 
  }

  //returns the positions of the swerve modules in an array 
  //getPosition uses drive enc and module rotation 
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4]; 

    for (SwerveModule swerveMod : swerveModules) {
      positions[swerveMod.moduleID] = swerveMod.getPosition();
    }

    return positions;
  }

  /* * * SET METHODS * * */
  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  //gets a SwerveModuleStates array from driver control and sets each module to the corresponding SwerveModuleState
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setState(desiredStates[swerveMod.moduleID]);
    }
  }

  //Overloaded for auto 
  public void setModuleStates(SwerveModuleState[] desiredStates, double speed) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setState(desiredStates[swerveMod.moduleID], speed);
    }
  }


  /* * * DRIVE METHODS * * */
  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented, double SpeedMultiplier){
    
    SwerveModuleState[] states;

    if (fieldOriented) {

      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed * SpeedMultiplier, ySpeed * SpeedMultiplier, zSpeed * SpeedMultiplier, getRotation2d())
      );

    } else {

      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        new ChassisSpeeds(xSpeed, ySpeed, zSpeed)
      );
      
    }



    // Double[] swerveArr = {states[0].angle.getDegrees(),states[1].angle.getDegrees(),states[2].angle.getDegrees(),states[3].angle.getDegrees()};

    // SmartDashboard.putNumberArray("swerveTest", swerveArr); //NOTE the thing

    setModuleStates(states);   

  }

  public void driveRobotRelative(ChassisSpeeds chassis) {

    SwerveModuleState[] state = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassis);

    //Limited for auto
    setModuleStates(state, 0.10);

  }

  /* * * WHEEL METHODS * * */
  public void lock() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    states[0] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)));
    states[1] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45)));
    states[2] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)));
    states[3] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45)));

    for (SwerveModule swerveMod : swerveModules) {
      System.out.println(swerveMod.moduleID);
      swerveMod.setAngle(states[swerveMod.moduleID]);
    }

  }

  public void straightenWheels() { //set all wheels to 0 degrees 
    SwerveModuleState[] states = new SwerveModuleState[4]; 

    states[0] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[1] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[2] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[3] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setState(states[swerveMod.moduleID]);
    }

  }

  public void stopModules() {

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.stop();
    }

}

  public void updateVisionOdometry() {

    boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if(mt1 == null){ //Incase of limelight disconnect
         return;
      }
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(1,1,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else if (useMegaTag2 == true)
    {
      // FIXME 180 reverse side thing
      LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      //LimelightHelpers.SetRobotOrientation("limelight", getRotation2d().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      /*if(Math.abs(pigeon.getan) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      { // TODO pigeon angular velocity
        doRejectUpdate = true;
      }*/

      if(mt2 == null){ //Incase of limelight disconnect
         return;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {

        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }

    
  }

//   public void updateOdometry() {
//     LimelightHelpers.SetRobotOrientation(
//         "limelight",
//         m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
//         0,
//         0,
//         0,
//         0,
//         0);

//     PoseEstimate estimate = limelight.getTrustedPose();
//     if (estimate != null) {
//       boolean doRejectUpdate = false;
//       if (Math.abs(pigeon.getAngularVelocityZWorld().getValueAsDouble()) > 720) {
//         doRejectUpdate = true;
//       }
//       if (estimate.tagCount == 0) {
//         doRejectUpdate = true;
//       }
//       if (!doRejectUpdate) {
//         odometer.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
//         RobotState.getInstance().LimelightsUpdated = true;
//       } else {
//         RobotState.getInstance().LimelightsUpdated = false;
//       }
//   }
// }

  @Override
  public void periodic() {
    LimelightHelpers.setLEDMode_ForceOff("limelight");
    
    // This method will be called once per scheduler run
    //updateVisionOdometry();

    m_poseEstimator.update(
        pigeon.getRotation2d(),
        getModulePositions());
    
    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.print();
    }

    SmartDashboard.putNumber("Pigeon", pigeon.getYaw().getValueAsDouble());
    m_field.setRobotPose(getPose());
  }
}