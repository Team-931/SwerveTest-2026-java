// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.SwvModConst;
import frc.robot.Constants.DrvConst.Setup;

public class SwerveModule {
  /* 
  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared
 */
  private final Setup info;
  private final double absOffset;
  private double relOffset;
  private final SparkMax driveMotor;
  private final SparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;
  private final AbsoluteEncoder absoluteEncoder;

  
  private final SparkClosedLoopController drivePIDController;

  
  private final SparkClosedLoopController turningPIDController;


  /**
   * Construct a SwerveModule from a Setup class
   */  
SwerveModule (Setup setup){
    info = setup;
    driveMotor = new SparkMax(setup.driveId, MotorType.kBrushless);
    turningMotor = new SparkMax(setup.turnId, MotorType.kBrushless);
    absOffset = setup.absOffset;

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();
    absoluteEncoder = turningMotor.getAbsoluteEncoder();

    drivePIDController = driveMotor.getClosedLoopController();
    turningPIDController =
      turningMotor.getClosedLoopController();

    SparkMaxConfig configDrv = new SparkMaxConfig(),
                  configTrn = new SparkMaxConfig();
      // TODO: check voltage comp
    configDrv.voltageCompensation(Constants.nominalVoltage);      
    configTrn.voltageCompensation(Constants.nominalVoltage);      
    configDrv.encoder.positionConversionFactor(SwvModConst.driveConversion)       // New unit: meters
                  .velocityConversionFactor(SwvModConst.driveConversion / 60); // New unit: meters / second
    configDrv.closedLoop.p(SwvModConst.posP / SwvModConst.driveConversion, SwvModConst.posSlot)
                     //.p(SwvModConst.velP / SwvModConst.driveConversion * 60, SwvModConst.velSlot)
                     .i(SwvModConst.velI, SwvModConst.velSlot) // This compensates for inaccuracy in feed-forward
                     .iZone(SwvModConst.velIZone, SwvModConst.velSlot) // When we change veloc. by more than this let feed-forward do the work
                     .feedForward.kV(SwvModConst.DrvFF, SwvModConst.velSlot); // about enough to reach the right veloc. without feed-back

    driveMotor.configure(configDrv, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    configTrn.encoder.positionConversionFactor(SwvModConst.turnConversion)       // New unit: radians
                  .velocityConversionFactor(SwvModConst.turnConversion / 60); // New unit: radians / second
    configTrn.absoluteEncoder.positionConversionFactor(1. /3.3)       // New unit: radians
                  .velocityConversionFactor(1. / 60 / 3.3); // New unit: radians / second
    configTrn.closedLoop.p(SwvModConst.posP / SwvModConst.turnConversion, SwvModConst.posSlot) // main control for angle
                     .i(SwvModConst.turnI, SwvModConst.posSlot) // overcome resistance at small error
                     .iZone(SwvModConst.turnIZone, SwvModConst.posSlot) // ignore .i for larger error
                     .p(SwvModConst.velP / SwvModConst.turnConversion * 60, SwvModConst.velSlot)
                     .positionWrappingEnabled(true)
                     .positionWrappingInputRange(-.25, .25)
                     .feedForward.kV(0, SwvModConst.velSlot); // Todo:  check this

    turningMotor.configure(configTrn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    setRelOffset();
}

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   */
 
  private final void setRelOffset() {
    relOffset = absoluteEncoder.getPosition() - absOffset - turningEncoder.getPosition();
   }

  private final double turnRots() {
    return turningEncoder.getPosition() + relOffset;
  }
  
  private final Rotation2d turnAngle() {
    return Rotation2d.fromRotations(turnRots());
    }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity(), turnAngle());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), turnAngle());
  }

  void report() {
    SmartDashboard.putNumber(info.name + " angle", turnRots());
    SmartDashboard.putNumber(info.name + " speed", driveEncoder.getVelocity());
    SmartDashboard.putNumber(info.name + " setpoint", X);
    SmartDashboard.putNumber(info.name + " abs. angle", absoluteEncoder.getPosition());
  }

  void fullSpeed() {
    driveMotor.set(1);
  }
  
private boolean noLaborSaving = false;

  void doAngle360(boolean yes) {//TODO: don't need after abs encoders are in
    noLaborSaving = yes;
    double range = yes ? .5: .25;
    var config = new SparkMaxConfig(); config.closedLoop.positionWrappingInputRange(-range, range);
    turningMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private double X;//moved out of setVel only for reporting purpose

  //private Translation2d velGoal = new Translation2d();
  public void setVel(Translation2d translation2d, double period) {
    Translation2d velGoal = translation2d.rotateBy(turnAngle().unaryMinus());
    X = velGoal.getX() == 0 ? 1e-10 : velGoal.getX();//TODO: make X local again
    drivePIDController.setSetpoint(X, ControlType.kVelocity, SwvModConst.velSlot);//Done: check conversion factors

    if (Robot.useVelCtrl) {
//      SmartDashboard.putNumber("slope", velGoal.getY()/X);
      turningPIDController.setSetpoint(velGoal.getY()/X / period, ControlType.kVelocity, SwvModConst.velSlot);
    }
    else if(noLaborSaving || translation2d.getSquaredNorm() >= (1e-6)) {// square of 1 mm / sec
      double angle = Math.atan2(translation2d.getY(), translation2d.getX()) / 2 / Math.PI;
      turningPIDController.setSetpoint(angle, ControlType.kPosition, SwvModConst.posSlot);
    }
  }
 
}
