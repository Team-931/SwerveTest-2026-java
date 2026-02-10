package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    static final double deadBand = .05;
  
    public final class DrvConst {
         static final double kMaxSpeed = 3.0, overloadSpeed = kMaxSpeed/* or SwvModConst.freeVeloc */; // 3 meters per second
         static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
        // CAN IDs for the motor controllers, must match the controller's setup
         static final int BRTrn = 7, BRDrv = 8,
                          BLTrn = 4, BLDrv = 2,
                          FRTrn = 6, FRDrv = 9,
                          FLTrn = 5, FLDrv = 3;
  
         static final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381); // unit: meters; x is forward dist from center, y is leftward
         static final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
         static final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
         static final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
         
         static final Rotation2d ClockW90 = new Rotation2d(0, 1);
         static final Translation2d m_frontLeftClW = m_frontLeftLocation.rotateBy(ClockW90); 
         static final Translation2d m_frontRightClW = m_frontRightLocation.rotateBy(ClockW90);
         static final Translation2d m_backLeftClW = m_backLeftLocation.rotateBy(ClockW90);
         static final Translation2d m_backRightClW = m_backRightLocation.rotateBy(ClockW90);
         
         static final double driveRadius = 
                 Math.max(m_frontLeftLocation.getNorm(), Math.max(m_frontRightLocation.getNorm(), 
                 Math.max(m_backLeftLocation.getNorm(), m_backRightLocation.getNorm())));
		//static final double IMUAngle = -90; // Angle (degrees clockwise) of navX logo: right-side-up would be 0 as seen from back

    }
    public final class SwvModConst {
        final static ClosedLoopSlot posSlot = ClosedLoopSlot.kSlot0;
        final static double posP = 2. / 3;
        final static ClosedLoopSlot velSlot = ClosedLoopSlot.kSlot1;
        final static double velP = .0001;
        static final double kWheelRadius = 0.034; //meter
        static final double freeVeloc = 5.31;
         //TODO: tune better
        static final double DrvFF = 1 / freeVeloc; // Officially Volt /(m/s), conjectured: proportional output / (m/s)
        static final int turnGearing = 28, driveGearing = 4;
        static final double driveConversion = 2 * Math.PI * kWheelRadius / driveGearing, // motor rotations to output meters
                            turnConversion = 2 * Math.PI / turnGearing;
        static final double velI = 0.001, velIZone = .05;
    }
    static final int nominalVoltage = 12;
    
}
