// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrvConst;

public class Robot extends TimedRobot {
  private final XboxController drive_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();
  private final transferShooter actualname = new transferShooter();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // Report swerve drive data
  {addPeriodic(m_swerve::report, .25);}
  private final Timer autoTimer = new Timer();
  @Override
  public void autonomousInit() {
    autoTimer.restart();
  }
  @Override
  public void autonomousPeriodic() {
    double xSpeed = (autoTimer.get() < 2) ? 1 : 0; //Drive fwd 1 m/s for 2 s
    m_swerve.drive(xSpeed, 0, 0, useField, getPeriod());
    //driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  static boolean useField = true, useVelCtrl = false;

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putBoolean("vision target found", LimelightHelpers.getTV("limelight-a"));
    SmartDashboard.putBoolean("april tag found", LimelightHelpers.getTV("limelight-b"));
    driveWithJoystick(useField);
    m_swerve.updateOdometry();
    if (drive_controller.getLeftStickButtonPressed()) actualname.shoot();

    //if(drive_controller.getBButton()){System.out.println("Hello world");}
    
  }

  private boolean firstTimeDisabled = true;

  @Override
  public void disabledInit() {
    if (firstTimeDisabled)
    {
      firstTimeDisabled = false;
      //m_swerve.zeroYaw();//TODO: check if need
      showFieldCtr();
    }
    LimelightHelpers.setLEDMode_ForceOff("limelight-b");
    m_swerve.setRelOffset();
  }

  private double maxSpeed = DrvConst.kMaxSpeed,
                maxAngularSpeed = DrvConst.kMaxAngularSpeed;

  void showMaxSpeeds() {
    SmartDashboard.putNumber("Max. linear speed", maxSpeed);
    SmartDashboard.putNumber("Max. angular speed", maxAngularSpeed);
  }

  void setMaxSpeed(double speed) {
    maxSpeed = speed;
  }
  
  void setMaxAngularSpeed(double speed) {
    maxAngularSpeed = speed;
  }
  
  private void showFieldCtr() {
    SmartDashboard.putBoolean("Field Centered", useField);
  }
  private void driveWithJoystick(boolean fieldRelative) {
    if(drive_controller.getRightBumperButtonPressed()) m_swerve.doAngle360(true); //TODO: don't need after abs encoders are in
    if(drive_controller.getRightBumperButtonReleased()) m_swerve.doAngle360(false); //TODO: don't need after abs encoders are in,
    if(drive_controller.getLeftBumperButtonPressed()) m_swerve.setXPosture(getPeriod());
    if(drive_controller.getAButtonPressed()) m_swerve.zeroYaw(); /* useVelCtrl ^= true; */
    if(drive_controller.getBButtonPressed()) {
      useField ^= true;
      showFieldCtr();
    }
    if(drive_controller.getXButton()) {
      m_swerve.fullSpeed();
      return;
    }
    if(drive_controller.getYButton()) {
      m_swerve.drive(1, 0, 0, false, getPeriod());
      return;
    }
    // DONE: have max speed modifiable
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        - m_xspeedLimiter.calculate(MathUtil.applyDeadband(drive_controller.getLeftY(), Constants.deadBand))
            * maxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        - m_yspeedLimiter.calculate(MathUtil.applyDeadband(drive_controller.getLeftX(), Constants.deadBand))
            * maxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        - m_rotLimiter.calculate(MathUtil.applyDeadband(drive_controller.getRightX(), Constants.deadBand))
            * maxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
}
