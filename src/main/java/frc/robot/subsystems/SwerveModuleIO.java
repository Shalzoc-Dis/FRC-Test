package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModuleIO{
  
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, int number) {
  }

  public void resetToAbsolute() {
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
  }

  private void setAngle(SwerveModuleState desiredState) {
  }

  public Rotation2d getCanCoder() {
    return null;
  }

  public SwerveModuleState getState() {
    return null;
  }

  public SwerveModulePosition getPostion() {
    return null;
  }

  public SwerveModuleState xState(){
    return null;
  }
}