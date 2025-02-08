package frc.robot.subsystems;

import frc.robot.Constants.SwerveConstants;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.Pigeon2;

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

  }

  /* * * RESET METHODS * * */
  public void resetPigeon() {
    pigeon.reset();
  }  
  
  public void resetOdometry() {
    m_poseEstimator.resetPosition(getRotation2d(), getModulePositions(), new Pose2d());
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

  /* * * DRIVE METHODS * * */
  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented, double SpeedMultiplier){
    
    SwerveModuleState[] states;

    if (fieldOriented) {

      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed * SpeedMultiplier, ySpeed * SpeedMultiplier, zSpeed, getRotation2d())
      );

    } else {

      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        new ChassisSpeeds(xSpeed, ySpeed, zSpeed)
      );
      
    }

    setModuleStates(states);   

  }

  public void driveRobotRelative(ChassisSpeeds chassis) {

    SwerveModuleState[] state = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassis);

    setModuleStates(state);

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

  @Override
  public void periodic() {
    /*m_poseEstimator.update(
        pigeon.getRotation2d(),
        getModulePositions());
    // This method will be called once per scheduler run
    odometer.update(pigeon.getRotation2d(), getModulePositions());*/
    
    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.print();
    }

    /*LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if(limelightMeasurement.tagCount >= 1)
    {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      m_poseEstimator.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds - 3);
    }*/

    SmartDashboard.putNumber("Pigeon", pigeon.getYaw().getValueAsDouble());
    //putString("POSE INFO", m_poseEstimator.toString());
    //m_field.setRobotPose(getPose());
  }
}