// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.DrvConst;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  private final SwerveModule m_frontLeft = new SwerveModule(DrvConst.FLDrv, DrvConst.FLTrn);
  private final SwerveModule m_frontRight = new SwerveModule(DrvConst.FRDrv, DrvConst.FRTrn);
  private final SwerveModule m_backLeft = new SwerveModule(DrvConst.BLDrv, DrvConst.BLTrn);
  private final SwerveModule m_backRight = new SwerveModule(DrvConst.BRDrv, DrvConst.BRTrn);

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
    
  Translation2d FLVel, BLVel, FRVel, BRVel,  
  BaseVel = new Translation2d(xSpeed, ySpeed);
  double maxSpeed = BaseVel.getNorm() + DrvConst.driveRadius * Math.abs(rot);
  if (maxSpeed > DrvConst.overloadSpeed) {
    BaseVel = BaseVel.times(DrvConst.overloadSpeed / maxSpeed);
    rot *= (DrvConst.overloadSpeed / maxSpeed);
  }

  if(fieldRelative) {
    BaseVel = BaseVel.rotateBy(m_gyro.getRotation2d().unaryMinus());
  }
  FLVel = BaseVel.plus(DrvConst.m_frontLeftClW.times(rot));
  BLVel = BaseVel.plus(DrvConst.m_backLeftClW.times(rot));
  FRVel = BaseVel.plus(DrvConst.m_frontRightClW.times(rot));
  BRVel = BaseVel.plus(DrvConst.m_backRightClW.times(rot));
  
  m_frontLeft.setVel(FLVel, periodSeconds);
  m_backLeft.setVel(BLVel, periodSeconds);
  m_frontRight.setVel(FRVel, periodSeconds);
  m_backRight.setVel(BRVel, periodSeconds);
}

void fullSpeed() {
  m_backLeft.fullSpeed();
  m_backRight.fullSpeed();
  m_frontLeft.fullSpeed();
  m_frontRight.fullSpeed();
}

   void report() {
    m_frontLeft.report("FL");
    m_frontRight.report("FR");
    m_backLeft.report("BL");
    m_backRight.report("BR");
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
