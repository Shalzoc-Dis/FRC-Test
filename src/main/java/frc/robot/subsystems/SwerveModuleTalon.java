package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.lib.Items.Controllers.TalonController;
import frc.lib.configs.Subsystems.TalonModuleInfo;
import frc.lib.math.OnboardModuleState;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.networktables.GenericEntry;

import java.util.Map;

public class SwerveModuleTalon extends SwerveModuleIO{
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;
  private GenericEntry angleEntry;

  private TalonController angle;
  private TalonController drive;

  private TalonFX angleMotor;
  private TalonFX driveMotor;

  private CANcoder angleEncoder;

  public final SwerveModuleState xState;

  private boolean isAbsolute = false;

  private double driveConvert;
  private double angleConvert;
  private ShuffleboardTab display = Shuffleboard.getTab("Testing");
  
  public SwerveModuleTalon(TalonModuleInfo Info) {
        angleEntry = display.add("Module " + Info.moduleNumber, 0)
                                .withWidget(BuiltInWidgets.kGraph)
                                .getEntry();

    this.moduleNumber = Info.moduleNumber;
    this.angleOffset = Rotation2d.fromDegrees(Info.angleOffset);

    xState = new SwerveModuleState(0, Rotation2d.fromDegrees(Info.xPos));

    angle = Info.angle;
    drive = Info.drive;

    /* Angle Encoder Config */
    angleEncoder = Info.cancoder;

    angleMotor = angle.talon;
    driveMotor = drive.talon;

    lastAngle = getState().angle;

    driveConvert = Info.driveConvert;
    angleConvert = Info.angleConvert;
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, int number) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

      setAngle(desiredState, number);
      setSpeed(desiredState, isOpenLoop);
  }

  
  public void resetToAbsolute() {
    if(isAbsolute){
      return;
    }
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    VoltageOut target = new VoltageOut(0.0);
    angleMotor.setPosition(ConvertAngleIn(Units.degreesToRotations(absolutePosition)));
    angleMotor.setControl(target);
    SmartDashboard.putNumber("PassedAngle" + moduleNumber, absolutePosition);
    isAbsolute = true;
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    }
    }
  private void setAngle(SwerveModuleState desiredState, int number) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    // Rotation2d angle =
    //     (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
    //         ? lastAngle
    //         : desiredState.angle;
  
    Rotation2d angle = desiredState.angle;
  
    angleEntry.setDouble(angle.getRotations());

    PositionVoltage target = new PositionVoltage(0);
    angleMotor.setControl(target.withPosition(ConvertAngleIn(Units.degreesToRotations(angle.getDegrees()))));
    lastAngle = angle;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public Rotation2d getCanCoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(), getAngle());
  }

  @Override
  public SwerveModulePosition getPostion() {
    return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble(), getAngle());
  }

  @Override
  public SwerveModuleState xState(){
    return xState;
  }

  // Coverts m/s to rotations/s
  private double ConvertDriveIn(double angle){
    return angle / driveConvert;
  }

  // Coverts m/s to rotations/s
  private double ConvertAngleIn(double angle){
    return angle / angleConvert;
  }

  // Coverts m/s to rotations/s
  private double ConvertDriveOut(double angle){
    return angle * driveConvert;
  }
  
  // Coverts m/s to rotations/s
  private double ConvertAngleOut(double angle){
    return angle * angleConvert;
  }
  
  private void logNumber(ShuffleboardTab tab, String name, double number) {
    tab.add(name, number)
       .withWidget(BuiltInWidgets.kGraph)
       .getEntry();
  }

}