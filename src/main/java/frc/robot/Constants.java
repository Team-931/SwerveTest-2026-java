package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

final class Constants {
    static final double deadBand = .05;
  
    static final class DrvConst {
         static final double kMaxSpeed = 3.0, overloadSpeed = kMaxSpeed/* or SwvModConst.freeVeloc */; // 3 meters per second
         static final double kMaxAngularSpeed = .5; // 1/2 radian per second
         static final class Setup {
            final int driveId, turnId;
            final double absOffset;
            final String name;
            Setup(int drv, int trn, double offset, String nam3) {
                driveId = drv;
                turnId = trn;
                absOffset = offset;
                name = nam3;
            }
         }

         static final double baseOffset = .1875; // .18611111111111111111111111111111
         static Setup frontLeft = new Setup(3, 5, baseOffset + .75, "FL"),
                     frontRight = new Setup(9, 6, baseOffset, "FR"),
                     backLeft = new Setup(2, 4, baseOffset + .5, "BL"),
                     backRight = new Setup(8, 7, baseOffset + .25, "BR");
        
         static final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381); // unit: meters; x is forward dist from center, y is leftward
         static final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
         static final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
         static final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);
         
         static final Rotation2d ClockW90 = new Rotation2d(0, 1);
         static final Translation2d frontLeftClW = frontLeftLocation.rotateBy(ClockW90); 
         static final Translation2d frontRightClW = frontRightLocation.rotateBy(ClockW90);
         static final Translation2d backLeftClW = backLeftLocation.rotateBy(ClockW90);
         static final Translation2d backRightClW = backRightLocation.rotateBy(ClockW90);
         
         static final double driveRadius = 
                 Math.max(frontLeftLocation.getNorm(), Math.max(frontRightLocation.getNorm(), 
                 Math.max(backLeftLocation.getNorm(), backRightLocation.getNorm())));

    }
    static final class SwvModConst {
        final static ClosedLoopSlot posSlot = ClosedLoopSlot.kSlot0;
        final static double posP = .5;
        final static ClosedLoopSlot velSlot = ClosedLoopSlot.kSlot1;
        final static double velP = .0001;
        static final double kWheelRadius = .05931 / 2; /* 0.034 */; //meter //diameter: 2.335 in, 59.31 mm
        static final double freeVeloc = 4.63; //5.31;
         //TODO: tune better
        static final double DrvFF = 1 / freeVeloc; // Officially Volt /(m/s), conjectured: proportional output / (m/s)
        static final int turnGearing = 28, driveGearing = 4;
        static final double driveConversion = 2 * Math.PI * kWheelRadius / driveGearing, // motor rotations to output meters
        // TODO Decide whether turn unit should be radians, or rotations as currently.
                            turnConversion = 1. / turnGearing;
        static final double velI = 0.001, velIZone = .05;
        static final double turnI = 0.003 * posP, turnIZone = 1. / 64;
        static final double minSpd = .001, // mm / s
                            minSpdSq = minSpd*minSpd;
    }
    static final int nominalVoltage = 12;
    
}
