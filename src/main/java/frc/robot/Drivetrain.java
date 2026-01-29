// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrvConst;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  private final SwerveModule m_frontLeft = new SwerveModule(3, 5);
  private final SwerveModule m_frontRight = new SwerveModule(9, 6);
  private final SwerveModule m_backLeft = new SwerveModule(2, 4);
  private final SwerveModule m_backRight = new SwerveModule(8, 7);

  private final  AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  public Drivetrain() {
    m_gyro.reset();
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
    
  SmartDashboard.putNumber("FL angle", m_frontLeft.getPosition().angle.getDegrees());
  SmartDashboard.putNumber("FR angle", m_frontRight.getPosition().angle.getDegrees());
  SmartDashboard.putNumber("BL angle", m_backLeft.getPosition().angle.getDegrees());
  SmartDashboard.putNumber("BR angle", m_backRight.getPosition().angle.getDegrees());

  Translation2d FLVel, BLVel, FRVel, BRVel,  
  BaseVel = new Translation2d(xSpeed, ySpeed);
  double maxSpeed = BaseVel.getNorm() + DrvConst.driveRadius * Math.abs(rot);
  if (maxSpeed > DrvConst.overloadSpeed) {
    BaseVel = BaseVel.times(DrvConst.overloadSpeed / maxSpeed);
    rot *= (DrvConst.overloadSpeed / maxSpeed);
  }

  if(fieldRelative) {
    var orientationCorrection = m_gyro.getRotation2d().unaryMinus();
    BaseVel = BaseVel.rotateBy(orientationCorrection);
  }
  FLVel = BaseVel.plus(DrvConst.m_frontLeftLocation.times(rot));
  BLVel = BaseVel.plus(DrvConst.m_backLeftLocation.times(rot));
  FRVel = BaseVel.plus(DrvConst.m_frontRightLocation.times(rot));
  BRVel = BaseVel.plus(DrvConst.m_backRightLocation.times(rot));
  
  m_frontLeft.setVel(FLVel, periodSeconds);
  m_backLeft.setVel(BLVel, periodSeconds);
  m_frontRight.setVel(FRVel, periodSeconds);
  m_backRight.setVel(BRVel, periodSeconds);
}

  /** Updates the field relative position of the robot. */
/*   public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }
 */}
