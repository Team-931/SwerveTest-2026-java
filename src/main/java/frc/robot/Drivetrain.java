// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  private final Rotation2d ClockW90 = new Rotation2d(0, 1);
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381).rotateBy(ClockW90);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381).rotateBy(ClockW90);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381).rotateBy(ClockW90);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381).rotateBy(ClockW90);

  private final SwerveModule m_frontLeft = new SwerveModule(3, 5, false);
  private final SwerveModule m_frontRight = new SwerveModule(9, 6, true);
  private final SwerveModule m_backLeft = new SwerveModule(2, 4, false);
  private final SwerveModule m_backRight = new SwerveModule(8, 7, true);

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
  if(fieldRelative) {
    var orientationCorrection = m_gyro.getRotation2d().unaryMinus();
    BaseVel = BaseVel.rotateBy(orientationCorrection);
  }
  FLVel = BaseVel.plus(m_frontLeftLocation.times(rot));
  BLVel = BaseVel.plus(m_backLeftLocation.times(rot));
  FRVel = BaseVel.plus(m_frontRightLocation.times(rot));
  BRVel = BaseVel.plus(m_backRightLocation.times(rot));
  
  m_frontLeft.setVel(FLVel);
  m_backLeft.setVel(BLVel);
  m_frontRight.setVel(FRVel);
  m_backRight.setVel(BRVel);
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
