package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    static final double deadBand = .05;
  
    public final class DrvConst {
         static final double kMaxSpeed = 3.0, overloadSpeed = kMaxSpeed; // 3 meters per second
         static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

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

    }
    public final class SwvModConst {
        public final static ClosedLoopSlot posSlot = ClosedLoopSlot.kSlot0;
        public final static double posP = .1;
        public final static ClosedLoopSlot velSlot = ClosedLoopSlot.kSlot1;
        public final static double velP = .0001;
        public static final double kWheelRadius = 0.0508;
        static final int turnGearing = 28, driveGearing = 4;
        static final double driveConversion = 2 * Math.PI * kWheelRadius / driveGearing, // motor rotations to output meters
                            turnConversion = 2 * Math.PI / turnGearing;
    }
    
}
