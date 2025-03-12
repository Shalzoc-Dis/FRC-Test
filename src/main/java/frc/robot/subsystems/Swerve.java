package frc.robot.subsystems;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.configs.Subsystems.SparkModuleInfo;
import frc.lib.configs.Subsystems.TalonModuleInfo;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private final AHRS gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModuleIO[] mSwerveMods;

  private boolean isX = false;

  private boolean negativePitch = false;

  private Field2d field = new Field2d();

  public Swerve() {
    gyro = new AHRS(NavXComType.kMXP_SPI);
    gyro.reset();
    SmartDashboard.putBoolean("GyroConnected", gyro.isConnected());
    zeroGyro();

    mSwerveMods = new SwerveModuleIO[4];

    //Asserts which Module controller constructer will be utilized
    if(Constants.Swerve.m_motorType == Constants.Swerve.motorType.spark){
    for(int i = 0; i <= 3; i++){
        mSwerveMods[i] = new SwerveModuleRev(new SparkModuleInfo(i));
    }
    } else if(Constants.Swerve.m_motorType == Constants.Swerve.motorType.talon){
      for(int i = 0; i <= 3; i++){
        mSwerveMods[i] = new SwerveModuleTalon(new TalonModuleInfo(i));
    } 
    }
    
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getAngle(), getPositions());
    
    RobotConfig config;   
    
    try {
      config = RobotConfig.fromGUISettings();
    } catch (IOException | ParseException e) {
      config = Constants.Swerve.config;
    };
  

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            Constants.Swerve.pathFollowerConfig, // The robot configuration
            config,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
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

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic(){
    report();
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getAngle())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
      for (int i = 0; i < mSwerveMods.length; i++) {
        if(isX){
          mSwerveMods[i].setDesiredState(mSwerveMods[i].xState(), isOpenLoop, i);
        } else {
          mSwerveMods[i].setDesiredState(swerveModuleStates[i], isOpenLoop, i);
        }
      var modState = swerveModuleStates[i];
      SmartDashboard.putNumber("Mod " + i + " desired angle: ", modState.angle.getDegrees());
      SmartDashboard.putNumber("Mod " + i + " desired velocity: ", modState.speedMetersPerSecond);
    }
    swerveOdometry.update(getAngle(), getPositions());
  }

  public void xPattern(){
    isX = !isX;
  }

  public void xPatternTrue(){
    isX = true;
  }

  public void xPatternFalse(){
    isX = false;
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (int i = 0; i < mSwerveMods.length; i++) {
      mSwerveMods[i].setDesiredState(desiredStates[i], false, i);
    }
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getAngle(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < mSwerveMods.length; i++) {
      states[i] = mSwerveMods[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < mSwerveMods.length; i++) {
      positions[i] = mSwerveMods[i].getPostion();
    }
    return positions;
  }

  public ChassisSpeeds getSpeeds(){
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
  }

  public void zeroGyro() {
    gyro.zeroYaw();
    gyro.setAngleAdjustment(0);
    negativePitch = false;
  }

  public Rotation2d getAngle() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getAngle())
        : Rotation2d.fromDegrees(gyro.getAngle());
  }

  public void resetToAbsolute() {
    for (SwerveModuleIO mod : mSwerveMods) {
        mod.resetToAbsolute();
    }
  }

  public float getPitch(){
    if (negativePitch){
      return -gyro.getPitch();
    } else {
      return gyro.getPitch();
    }
  }

  public void invertGyro(){
    gyro.setAngleAdjustment(180);
    negativePitch = true;
  }

  public AHRS getGyro(){
    return gyro;
  }

  public void report(){
    for (int i = 0; i < mSwerveMods.length; i++) {
        SmartDashboard.putNumber(
            "Mod " + i + " Cancoder", mSwerveMods[i].getCanCoder().getDegrees());
        SmartDashboard.putNumber(
            "Mod " + i + " Integrated", mSwerveMods[i].getState().angle.getDegrees());
        SmartDashboard.putNumber(
            "Mod " + i + " Velocity", mSwerveMods[i].getState().speedMetersPerSecond);      
      }
    
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    SmartDashboard.putBoolean("IsX", isX);
}
}