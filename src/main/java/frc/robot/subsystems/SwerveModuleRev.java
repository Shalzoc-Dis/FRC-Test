package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.lib.Items.Controllers.SparkController;
import frc.lib.configs.Subsystems.SparkModuleInfo;
import frc.lib.math.OnboardModuleState;

public class SwerveModuleRev extends SwerveModuleIO{
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private SparkController drive;
  private SparkController angle;

  private SparkMax angleMotor;
  private SparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController angleController;

  public final SwerveModuleState xState;

  private boolean isAbsolute = false;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveMotorsSVA[0], Constants.Swerve.driveMotorsSVA[1], Constants.Swerve.driveMotorsSVA[2]);

  public SwerveModuleRev(SparkModuleInfo Info) {
    this.moduleNumber = Info.moduleNumber;
    this.angleOffset = Rotation2d.fromDegrees(Info.angleOffset);

    this.drive = Info.drive;
    this.angle = Info.angle;

    xState = new SwerveModuleState(0, Rotation2d.fromDegrees(Info.xPos));

    /* Angle Encoder Config */
    angleEncoder = Info.cancoder;

    /* Angle Motor Config */
    angleMotor = angle.spark;
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getClosedLoopController();

    /* Drive Motor Config */
    driveMotor = drive.spark;
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();

    lastAngle = getState().angle;
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, int number) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

      setAngle(desiredState);
      setSpeed(desiredState, isOpenLoop);
  }

  
  public void resetToAbsolute() {
    if(isAbsolute){
      return;
    }
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    angleController.setReference(0, ControlType.kDutyCycle);
    integratedAngleEncoder.setPosition(absolutePosition);
    SmartDashboard.putNumber("PassedAngle" + moduleNumber, absolutePosition);
    isAbsolute = true;
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  @Override
  public Rotation2d getCanCoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  @Override
  public SwerveModulePosition getPostion() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  @Override
  public SwerveModuleState xState(){
    return xState;
  }
}