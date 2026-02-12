// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.DrvConst;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  private final SwerveModule frontLeft = new SwerveModule(DrvConst.frontLeft);
  private final SwerveModule frontRight = new SwerveModule(DrvConst.frontRight);
  private final SwerveModule backLeft = new SwerveModule(DrvConst.backLeft);
  private final SwerveModule backRight = new SwerveModule(DrvConst.backRight);

  private final  AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  public Drivetrain() {
    gyro.reset();//TODO: Is this line needed?
    while(gyro.isCalibrating());
    zeroYaw();
  }

  void zeroYaw() {
    gyro.zeroYaw();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    
  Translation2d FLVel, BLVel, FRVel, BRVel,  
  BaseVel = new Translation2d(xSpeed, ySpeed);
  double maxSpeed = BaseVel.getNorm() + DrvConst.driveRadius * Math.abs(rot);
  if (maxSpeed > DrvConst.overloadSpeed) {
    BaseVel = BaseVel.times(DrvConst.overloadSpeed / maxSpeed);
    rot *= (DrvConst.overloadSpeed / maxSpeed);
  }

  if(fieldRelative) {
    BaseVel = BaseVel.rotateBy(gyro.getRotation2d().unaryMinus());
  }
  FLVel = BaseVel.plus(DrvConst.frontLeftClW.times(rot));
  BLVel = BaseVel.plus(DrvConst.backLeftClW.times(rot));
  FRVel = BaseVel.plus(DrvConst.frontRightClW.times(rot));
  BRVel = BaseVel.plus(DrvConst.backRightClW.times(rot));
  
  frontLeft.setVel(FLVel, periodSeconds);
  backLeft.setVel(BLVel, periodSeconds);
  frontRight.setVel(FRVel, periodSeconds);
  backRight.setVel(BRVel, periodSeconds);
}

void fullSpeed() {
  backLeft.fullSpeed();
  backRight.fullSpeed();
  frontLeft.fullSpeed();
  frontRight.fullSpeed();
}

  void doAngle360(boolean yes) {//TODO: don't need after abs encoders are in
    backLeft.doAngle360(yes);
    backRight.doAngle360(yes);
    frontLeft.doAngle360(yes);
    frontRight.doAngle360(yes);
  }
   void report() {
    frontLeft.report();
    frontRight.report();
    backLeft.report();
    backRight.report();
  }

  /** Updates the field relative position of the robot. */
/*   public void updateOdometry() {
    m_odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
  }
 */}
