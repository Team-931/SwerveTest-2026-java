// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Module;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int turnGearing = 28, driveGearing = 4;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SparkClosedLoopController m_drivePIDController;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SparkClosedLoopController m_turningPIDController;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param reverse True if drive is opposed to its angle
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel
      ) {
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder();

    m_drivePIDController = m_driveMotor.getClosedLoopController();
    m_turningPIDController =
      m_turningMotor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();  
    config.encoder.positionConversionFactor(2 * Math.PI * kWheelRadius / driveGearing)       // New unit: metera
                  .velocityConversionFactor(2 * Math.PI * kWheelRadius / driveGearing / 60); // New unit: meters / second
    config.closedLoop.p(Module.posP / 2 / Math.PI / kWheelRadius * driveGearing, Module.posSlot)
                     .p(Module.velP / 2 / Math.PI / kWheelRadius * driveGearing * 60, Module.velSlot)
                     .positionWrappingEnabled(false);

    m_driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.encoder.positionConversionFactor(2 * Math.PI / turnGearing)       // New unit: radians
                  .velocityConversionFactor(2 * Math.PI / turnGearing / 60); // New unit: radians / second
    config.closedLoop.p(Module.posP / 2 / Math.PI * turnGearing, Module.posSlot)
                     .p(Module.velP / 2 / Math.PI * turnGearing * 60, Module.velSlot)
                     .positionWrappingEnabled(true)
                     .positionWrappingInputRange(-Math.PI/2, Math.PI/2); // Todo:  check this

    m_turningMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    //m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private final Rotation2d turnAngle() {return new Rotation2d(m_turningEncoder.getPosition());}

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), turnAngle());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), turnAngle());
  }

  //private Translation2d velGoal = new Translation2d();
  public void setVel(Translation2d translation2d, double period) {
    Translation2d velGoal = translation2d.rotateBy(turnAngle().unaryMinus());
    double X = velGoal.getX() == 0 ? 1e-10 : velGoal.getX();
    m_drivePIDController.setSetpoint(X, ControlType.kVelocity, Module.velSlot);//Todo: check conversion factors

    if (Robot.useVelCtrl) {
//      SmartDashboard.putNumber("slope", velGoal.getY()/X);
      m_turningPIDController.setSetpoint(velGoal.getY()/X / period, ControlType.kVelocity, Module.velSlot);
    }
    else {
      double angle = Math.atan2(translation2d.getY(), translation2d.getX());
      m_turningPIDController.setSetpoint(angle, ControlType.kPosition, Module.posSlot);
    }
  }
 
}
